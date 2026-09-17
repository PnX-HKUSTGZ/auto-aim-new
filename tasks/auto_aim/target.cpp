#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id{0},
  update_count_(0),
  armor_num_(armor_num),
  t_(t),
  is_switch_(false),
  is_converged_(false),
  switch_count_(0)
{
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z1 vz a w r l h1 h2
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h1: z2 - z1
  // h2: z3 - z1, only used by outpost
  Eigen::VectorXd x0{
    {center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0, 0}};  //初始化预测量
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
  ekf_.nis_gate_probability = 0.999;  // 拒绝门限与关联使用相同的 99.9% 分位点。
}

Target::Target(double x, double vyaw, double radius, double h1)
: name(ArmorName::one),
  armor_type(ArmorType::small),
  priority(ArmorPriority::first),
  jumped(false),
  armor_num_(4)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h1, 0}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
  ekf_.nis_gate_probability = 0.999;  // 拒绝门限与关联使用相同的 99.9% 分位点。
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  // 状态转移矩阵
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(12, 12);
  F(0, 1) = dt;
  F(2, 3) = dt;
  F(4, 5) = dt;
  F(6, 7) = dt;

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2;
  if (name == ArmorName::outpost) {
    v1 = 10;   // 前哨站加速度方差
    v2 = 0.1;  // 前哨站角加速度方差
  } else {
    v1 = 100;  // 加速度方差
    v2 = 400;  // 角加速度方差
  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差，r/l/h1/h2 按常量模型处理
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(12, 12);
  Q.block<2, 2>(0, 0) << a * v1, b * v1, b * v1, c * v1;
  Q.block<2, 2>(2, 2) << a * v1, b * v1, b * v1, c * v1;
  Q.block<2, 2>(4, 4) << a * v1, b * v1, b * v1, c * v1;
  Q.block<2, 2>(6, 6) << a * v2, b * v2, b * v2, c * v2;

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

  ekf_.predict(F, Q, f);
}

bool Target::update_armors(const std::list<Armor> & armors)
{
  const auto matched = match_armor_id(armors);
  if (!update_ypda(matched)) return false;

  // 仅接受的观测才能推进计数。板号集合比较不受两块观测输入顺序影响。
  std::vector<int> ids;
  for (const auto & item : matched) ids.push_back(item.second);
  auto previous = last_id;
  auto current = ids;
  std::sort(previous.begin(), previous.end());
  std::sort(current.begin(), current.end());
  is_switch_ = previous != current;
  if (is_switch_) ++switch_count_;
  jumped = jumped || ids.size() == 2 || ids.front() != 0;
  last_id = ids;
  ++update_count_;
  return true;
}

std::list<std::pair<Armor, int>> Target::match_armor_id(const std::list<Armor> & armors)
{
  // 目标实际板数：普通四板、前哨站三板；两者单帧均只融合一块或两块。
  // 遍历实际板数，不使用“固定最近三块”，关联同时考虑距离、高度和朝向。
  std::vector<const Armor *> candidates;
  std::vector<std::vector<double>> costs;
  double best_single = std::numeric_limits<double>::infinity();
  int single_observation = -1, single_id = -1; // 最优单板观测在 candidates 中的索引和板号
  for (const auto & armor : armors) {
    if (
      armor.name != name || armor.type != armor_type || !armor.xyz_in_world.allFinite() ||
      !armor.ypd_in_world.allFinite() || !armor.ypr_in_world.allFinite())
      continue;
    candidates.push_back(&armor);
    costs.emplace_back(armor_num_);
    for (int id = 0; id < armor_num_; ++id) {
      const double cost = tools::ExtendedKalmanFilter::innovation_nis( // 计算创新的归一化平方误差
        innovation(armor, id), h_jacobian(ekf_.x, id), ekf_.P, measurement_noise(armor));
      costs.back()[id] = cost;
      if (cost < best_single) {
        best_single = cost;
        single_observation = static_cast<int>(candidates.size()) - 1;
        single_id = id;
      }
    }
  }
  if (single_observation < 0) return {};
  std::list<std::pair<Armor, int>> best_pair;
  double best_joint = tools::ExtendedKalmanFilter::nis_threshold(8, ekf_.nis_gate_probability); // 联合代价门限
  const double single_gate =
    tools::ExtendedKalmanFilter::nis_threshold(4, ekf_.nis_gate_probability); // 单板代价门限

  // 多于两块视为额外候选/误检，仍只选最可信的一对，绝不做三板同时更新。
  for (size_t first = 0; first < candidates.size(); ++first) {
    for (size_t second = first + 1; second < candidates.size(); ++second) { // 遍历所有候选装甲板组合
      const auto & a = *candidates[first];
      const auto & b = *candidates[second];
      for (int i = 0; i < armor_num_; ++i) {
        for (int j = 0; j < armor_num_; ++j) { // 遍历所有板号组合
          if (i == j || costs[first][i] > single_gate || costs[second][j] > single_gate) continue;
          // 相对朝向消除公共转角不确定性，防止把重复检测硬配成两个不同板号。
          const double relative_error = tools::limit_rad(
            a.ypr_in_world[0] - b.ypr_in_world[0] - (i - j) * 2 * CV_PI / armor_num_);
          if (std::abs(relative_error) > CV_PI / armor_num_) continue; // 相对朝向差异过大，排除误配
          Eigen::MatrixXd H(8, 12); // 联合观测矩阵
          H.topRows(4) = h_jacobian(ekf_.x, i); // 上下拼接两块观测的线性化矩阵
          H.bottomRows(4) = h_jacobian(ekf_.x, j); 
          Eigen::MatrixXd R = Eigen::MatrixXd::Zero(8, 8); // 联合测量噪声协方差矩阵
          R.topLeftCorner(4, 4) = measurement_noise(a); // 上下拼接两块观测的测量噪声协方差矩阵
          R.bottomRightCorner(4, 4) = measurement_noise(b);
          Eigen::VectorXd residual(8); // 联合创新向量
          residual << innovation(a, i), innovation(b, j);
          // 联合代价保留 HPHᵀ 的交叉块，不能简单相加两块板各自的 NIS。
          const double cost = tools::ExtendedKalmanFilter::innovation_nis(residual, H, ekf_.P, R);
          if (cost < best_joint) {
            best_joint = cost;
            best_pair = {{a, i}, {b, j}};
          }
        }
      }
    }
  }
  if (!best_pair.empty()) return best_pair;
  // 没有可信双板组合时退回最优单板；是否接受由 EKF 门限决定并记录诊断。
  return {{*candidates[single_observation], single_id}};
}

Eigen::Vector4d Target::observation(const Eigen::VectorXd & x, int id) const // 获得预测的观测值
{
  const Eigen::Vector3d ypd = tools::xyz2ypd(h_armor_xyz(x, id));
  return {ypd[0], ypd[1], ypd[2], tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_)};
}

Eigen::Matrix4d Target::measurement_noise(const Armor & armor) const
{
  const double center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  const double delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  const Eigen::Vector4d diagonal{
    4e-3, 4e-3, std::log(std::abs(delta_angle) + 1) + 1,
    std::log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2};
  return diagonal.asDiagonal();
}

Eigen::Vector4d Target::innovation(const Armor & armor, int id) const
{
  Eigen::Vector4d residual = // 观测值 - 预测的观测值
    Eigen::Vector4d{
      armor.ypd_in_world[0], armor.ypd_in_world[1], armor.ypd_in_world[2], armor.ypr_in_world[0]} -
    observation(ekf_.x, id);
  for (int index : {0, 1, 3}) residual[index] = tools::limit_rad(residual[index]);
  return residual;
}

bool Target::update_ypda(const std::list<std::pair<Armor, int>> & matched_armors)
{
  // 每帧最多融合两块观测；双板必须对应不同板号。
  if (matched_armors.empty() || matched_armors.size() > 2) return false;
  if (matched_armors.size() == 2 && matched_armors.front().second == matched_armors.back().second)
    return false;

  const int dimension = 4 * matched_armors.size(); // 联合观测向量的维度
  Eigen::MatrixXd H(dimension, 12); // 联合观测矩阵
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dimension, dimension); // 联合测量噪声协方差矩阵
  Eigen::VectorXd z(dimension); // 联合观测向量
  int offset = 0;
  for (const auto & [armor, id] : matched_armors) {
    // 两块观测均在同一先验处线性化，H 上下拼接，R 为独立测量噪声的块对角阵。
    H.middleRows(offset, 4) = h_jacobian(ekf_.x, id); // 上下拼接两块观测的线性化矩阵
    R.block<4, 4>(offset, offset) = measurement_noise(armor); // 上下拼接两块观测的测量噪声协方差矩阵
    z.segment<4>(offset) << armor.ypd_in_world, armor.ypr_in_world[0]; // 上下拼接两块观测的观测值
    offset += 4;
  }
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd predicted(dimension);
    int offset = 0;
    for (const auto & [armor, id] : matched_armors) {
      predicted.segment<4>(offset) = observation(x, id); // 上下拼接两块观测的预测的观测值
      offset += 4;
    }
    return predicted;
  };
  auto subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd residual = a - b;
    for (int offset = 0; offset < residual.size(); offset += 4)
      for (int index : {0, 1, 3})
        residual[offset + index] = tools::limit_rad(residual[offset + index]);
    return residual;
  };
  ekf_.update(z, H, R, h, subtract);
  return ekf_.last_update_accepted;
}

bool Target::bad_quality() const
{
  // 至少积累 20 次观测，再按实际窗口长度判断；单帧异常由关联门限处理。
  const auto & failures = ekf_.recent_nis_failures;
  return failures.size() >= 20 &&
         std::accumulate(failures.begin(), failures.end(), 0) >= 0.4 * failures.size();
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

bool Target::diverged() const
{
  if (
    ekf_.x.size() != 12 || !ekf_.x.allFinite() || !ekf_.P.allFinite() ||
    (ekf_.P.diagonal().array() < 0).any())
    return true;
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

bool Target::convergened()
{
  // 保留现有最小更新次数，质量恶化时撤销收敛，避免前哨站错误持续锁定转速。
  const int required_updates = name == ArmorName::outpost ? 10 : 3;
  is_converged_ = update_count_ > required_updates && !diverged() && !bad_quality();

  return is_converged_;
}

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l = (armor_num_ == 4) && (id == 1 || id == 3);

  double dz_dh1 = 0.0;
  double dz_dh2 = 0.0;
  if (name == ArmorName::outpost) {
    dz_dh1 = (id == 1) ? 1.0 : 0.0;
    dz_dh2 = (id == 2) ? 1.0 : 0.0;
  } else if (use_l) {
    // 非前哨站保留原有的四装甲板交替高度模型，h2 固定为 0
    dz_dh1 = 1.0;
  }

  auto r = (use_l) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = x[4] + dz_dh1 * x[10] + dz_dh2 * x[11];

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l) ? -std::sin(angle) : 0.0;

  double dz_dh1 = 0.0;
  double dz_dh2 = 0.0;
  if (name == ArmorName::outpost) {
    dz_dh1 = (id == 1) ? 1.0 : 0.0;
    dz_dh2 = (id == 2) ? 1.0 : 0.0;
  } else if (use_l) {
    dz_dh1 = 1.0;
  }

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,      0,      0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,      0,      0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh1, dz_dh2},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,      0,      0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
