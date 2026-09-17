#include "planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
Planner::Planner(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
  comming_angle_ = tools::read<double>(yaml, "comming_angle") / 57.3;
  leaving_angle_ = tools::read<double>(yaml, "leaving_angle") / 57.3;
  air_resistance_ = yaml["air_resistance"].as<double>();
  fire_thresh_ = tools::read<double>(yaml, "fire_thresh");
  decision_speed_ = tools::read<double>(yaml, "decision_speed");
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");

  // 选板速度阈值（滞回），与延迟补偿的 decision_speed_ 解耦
  low_speed_thresh_ = tools::read<double>(yaml, "low_speed_thresh");
  high_speed_thresh_ = tools::read<double>(yaml, "high_speed_thresh");
  if (low_speed_thresh_ > high_speed_thresh_) std::swap(low_speed_thresh_, high_speed_thresh_);
  min_lock_hold_time_ = std::max(0.0, tools::read<double>(yaml, "min_lock_hold_time"));
  switch_margin_ = tools::read<double>(yaml, "switch_margin") / 57.3;

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}

Plan Planner::plan(Target target, double bullet_speed)
{
  const auto command_time = std::chrono::steady_clock::now();

  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22;
  }

  // 选板速度模式滞回：避免角速度在阈值附近反复切换
  const double w = std::abs(target.ekf_x()[7]);
  if (high_speed_mode_) {
    if (w < low_speed_thresh_) high_speed_mode_ = false;
  } else {
    if (w > high_speed_thresh_) high_speed_mode_ = true;
  }

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z(), air_resistance_);
  target.predict(bullet_traj.fly_time); // 预测飞行时间

  // 2. Get trajectory
  BoardSelectState st;
  st.lock_id = target.jumped ? lock_id_ : -1;  // 重建/新目标不继承旧板号
  st.lock_since = lock_since_;

  PlannerAimPoint commit_point;
  double yaw0;
  Trajectory traj;
  try { // 生成参考轨迹
    const auto command = aim(target, bullet_speed, st, command_time);
    commit_point = command.point;
    yaw0 = command.yaw_pitch(0);
    traj = get_trajectory(target, yaw0, bullet_speed, st, command, command_time);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    lock_id_ = -1;  // 目标丢失重置锁板
    lock_since_ = {};
    return {false};
  }

  // 只有命令时刻（t=0）的选板结果才提交到成员
  lock_since_ = st.lock_since;
  lock_id_ = commit_point.id;
  debug_aim_point_id_ = commit_point.id;
  debug_xyza = commit_point.xyza;

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_yaw_vel = traj(1, HALF_HORIZON);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  auto shoot_offset_ = 2;
  plan.fire =
    commit_point.can_fire && commit_point.id >= 0 &&
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  return plan;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed)
{
  if (!target.has_value()) {
    lock_id_ = -1;
    lock_since_ = {};
    return {false};
  }

  double delay_time =
    std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

  target->predict(future);

  return plan(*target, bullet_speed);
}

void Planner::setup_yaw_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
  auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
  auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  yaw_solver_->settings->max_iter = 10;
}

void Planner::setup_pitch_solver(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
  auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");
  auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  pitch_solver_->settings->max_iter = 10;
}

Planner::AimResult Planner::aim(
  const Target & target, double bullet_speed, BoardSelectState & st,
  std::chrono::steady_clock::time_point time)
{
  auto aim_point = choose_aim_point(target, st, high_speed_mode_, time);
  if (!aim_point.valid) throw std::runtime_error("Invalid aim point!");

  auto xyza = aim_point.xyza;
  auto xyz = xyza.head<3>();
  auto aim_dist = xyza.head<2>().norm();

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory(bullet_speed, aim_dist, xyz.z(), air_resistance_);
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  // 世界系方位角从 +X 起算，云台零方向为 +Y，发送角需减去 90°
  return {
    {tools::limit_rad(azim - M_PI / 2 + yaw_offset_), bullet_traj.pitch + pitch_offset_}, aim_point};
}

PlannerAimPoint Planner::choose_aim_point(
  const Target & target, BoardSelectState & st, bool high_speed_mode,
  std::chrono::steady_clock::time_point time)
{
  const Eigen::VectorXd & ekf_x = target.ekf_x();
  const auto armor_xyza_list = target.armor_xyza_list();
  const int armor_num = static_cast<int>(armor_xyza_list.size());

  if (armor_num == 0) return {false, false, -1, {0, 0, 0, 0}};

  const double center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  std::vector<double> delta_list;
  delta_list.reserve(armor_num);
  for (const auto & xyza : armor_xyza_list)
    delta_list.emplace_back(tools::limit_rad(xyza[3] - center_yaw));

  const bool outpost = target.name == ArmorName::outpost;
  const double coming = outpost ? 90.0 / 57.3 : comming_angle_;
  const double leaving = outpost ? 60.0 / 57.3 : leaving_angle_;

  // 射击窗口：命中时刻朝向需在进入/离开角之间；与跟踪候选分离
  auto in_fire_window = [&](int id) {
    const double delta = delta_list[id];
    if (outpost || high_speed_mode) {
      // 正转由负角进入，越过正 leaving 后离开；反转取镜像窗口。
      if (std::abs(delta) > coming) return false;
      if (ekf_x[7] > 0) return delta < leaving;
      if (ekf_x[7] < 0) return delta > -leaving;
      return false;
    }
    return std::abs(delta) <= 60.0 / 57.3;
  };

  // 初始化阶段：跟踪固定 0 号板（优先观测板），但开火仍按窗口判断
  if (!target.jumped) {
    if (st.lock_id != 0) st.lock_since = time;
    st.lock_id = 0;
    return {true, in_fire_window(0), 0, armor_xyza_list[0]};
  }

  // 跟踪候选：|delta| <= 跟踪窗口，射击窗口之外的板也纳入，保证轨迹连续
  const double track_window = outpost ? coming : 60.0 / 57.3;
  std::vector<int> id_list;
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_list[i]) <= track_window) id_list.push_back(i);
  }

  if (id_list.empty()) {
    // 无候选：改用旋转中心连续跟踪，只关开火，不退出控制
    tools::logger()->warn("No trackable board!");
    st.lock_id = -1;
    st.lock_since = time;
    return {true, false, -1, {ekf_x[0], ekf_x[2], ekf_x[4], center_yaw}};
  }

  // 锁板：新板明显更优 + 最短保持时间才切换；始终记录实际选中 ID
  int best = *std::min_element(
    id_list.begin(), id_list.end(),
    [&](int a, int b) { return std::abs(delta_list[a]) < std::abs(delta_list[b]); });

  bool in_candidates =
    std::find(id_list.begin(), id_list.end(), st.lock_id) != id_list.end(); // 当前锁板仍在候选中
  bool should_switch = !in_candidates;  // 单候选时也走这里，锁定实际板且不清 -1
  if (!should_switch && best != st.lock_id) {
    should_switch = std::abs(delta_list[best]) + switch_margin_ < std::abs(delta_list[st.lock_id]) &&
                    std::chrono::duration<double>(time - st.lock_since).count() >= min_lock_hold_time_;
  }

  if (should_switch) {
    st.lock_id = best;
    st.lock_since = time;
  }

  return {true, in_fire_window(st.lock_id), st.lock_id, armor_xyza_list[st.lock_id]};
}

Trajectory Planner::get_trajectory(
  Target target, double yaw0, double bullet_speed, const BoardSelectState & command_state,
  const AimResult & command, std::chrono::steady_clock::time_point time)
{
  Trajectory traj;
  std::array<AimResult, HORIZON + 2> samples;
  auto sample_state = command_state;
  target.predict(-DT * (HALF_HORIZON + 1));

  for (int i = 0; i < HORIZON + 2; ++i) {
    const int offset = i - HALF_HORIZON - 1;
    if (offset == 0) {
      // 历史采样不能改写当前决策；未来预测从真实命令状态开始。
      samples[i] = command;
      sample_state = command_state;
    } else {
      const auto sample_time = time + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(offset * DT));
      samples[i] = aim(target, bullet_speed, sample_state, sample_time);
    }
    target.predict(DT);
  }

  for (int i = 0; i < HORIZON; ++i) { // 采样点 i 对应 traj 的第 i 列
    const auto & last = samples[i];
    const auto & cur = samples[i + 1];
    const auto & next = samples[i + 2];
    const bool same_board = next.point.id >= 0 && next.point.id == cur.point.id &&
                            cur.point.id == last.point.id;
    const double yaw_vel = same_board
      ? tools::limit_rad(next.yaw_pitch(0) - last.yaw_pitch(0)) / (2 * DT) : 0.0;
    const double pitch_vel = same_board
      ? (next.yaw_pitch(1) - last.yaw_pitch(1)) / (2 * DT) : 0.0;
    traj.col(i) << tools::limit_rad(cur.yaw_pitch(0) - yaw0), yaw_vel, cur.yaw_pitch(1), pitch_vel;
  }
  return traj;
}

}  // namespace auto_aim
