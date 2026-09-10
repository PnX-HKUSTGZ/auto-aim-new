// 大符拟合移植自 SZURPVision/RP-26Rune（MIT），许可见 docs/licenses/RP-26Rune.txt。
#include "big_rune_motion_estimator.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Cholesky>

#include <stdexcept>

#include <algorithm>
#include <cmath>
#include <limits>

#include "tools/logger.hpp"

namespace auto_buff
{
namespace
{
constexpr int kRuneBladeCount = 5;
constexpr double kSectorPhase = 2.0 * M_PI / kRuneBladeCount;

template<typename T>
void read_optional(const YAML::Node & node, const char * key, T & value)
{
  if (node && node[key]) value = node[key].as<T>();
}
}  // namespace

BigRuneMotionEstimator::BigRuneMotionEstimator(const std::string & config_path)
{
  const YAML::Node phase = YAML::LoadFile(config_path)["buff_phase"];
  read_optional(phase, "big_max_time_interval", config_.max_time_interval);
  read_optional(phase, "big_prepare_time", config_.prepare_time);
  read_optional(phase, "big_window_seconds", config_.window_seconds);
  read_optional(phase, "big_min_data_size", config_.min_data_size);
  read_optional(
    phase, "big_rotation_vote_max_interval", config_.rotation_vote_max_interval);
  read_optional(phase, "big_target_switch_threshold", config_.target_switch_threshold);
  read_optional(
    phase, "big_expected_linear_velocity", config_.expected_linear_velocity);
  read_optional(phase, "big_omega_lower_bound", config_.omega_lower_bound);
  read_optional(phase, "big_omega_upper_bound", config_.omega_upper_bound);
  read_optional(phase, "big_omega_init", config_.omega_init);
  read_optional(phase, "big_outer_max_iterations", config_.outer_max_iterations);
  read_optional(phase, "big_outer_tolerance", config_.outer_tolerance);
  read_optional(phase, "big_lambda_init", config_.lambda_init);
  read_optional(phase, "big_lambda_min", config_.lambda_min);
  read_optional(phase, "big_lambda_max", config_.lambda_max);
  read_optional(phase, "big_max_omega_step", config_.max_omega_step);
  read_optional(phase, "big_diff_eps_min", config_.diff_eps_min);
  read_optional(phase, "big_diff_eps_rel", config_.diff_eps_rel);
  read_optional(phase, "big_highest_time_weight", config_.highest_time_weight);
  read_optional(phase, "big_inner_tolerance", config_.inner_tolerance);
  read_optional(phase, "big_lowest_time_weight", config_.lowest_time_weight);
  read_optional(phase, "big_irls_iterations", config_.irls_iterations);
  read_optional(phase, "big_robust_scale_factor", config_.robust_scale_factor);
  read_optional(phase, "big_jump_window_seconds", config_.jump_window_seconds);
  read_optional(phase, "big_jump_min_span_seconds", config_.jump_min_span_seconds);
  read_optional(phase, "big_jump_min_samples", config_.jump_min_samples);
  read_optional(phase, "big_jump_reset_ratio", config_.jump_reset_ratio);

  config_.min_data_size = std::max(config_.min_data_size, 4);
  if (!(config_.omega_lower_bound > 0.0 &&
        config_.omega_upper_bound >= config_.omega_lower_bound &&
        std::isfinite(config_.omega_upper_bound) && std::isfinite(config_.omega_init) &&
        config_.lowest_time_weight > 0.0 &&
        config_.highest_time_weight >= config_.lowest_time_weight &&
        std::isfinite(config_.highest_time_weight) &&
        config_.lambda_min > 0.0 && config_.lambda_max >= config_.lambda_min &&
        std::isfinite(config_.lambda_max) && std::isfinite(config_.lambda_init) &&
        config_.max_omega_step > 0.0 && std::isfinite(config_.max_omega_step) &&
        config_.diff_eps_min > 0.0 && std::isfinite(config_.diff_eps_min) &&
        config_.diff_eps_rel > 0.0 && std::isfinite(config_.diff_eps_rel) &&
        config_.inner_tolerance > 0.0 && std::isfinite(config_.inner_tolerance) &&
        config_.outer_tolerance > 0.0 && std::isfinite(config_.outer_tolerance) &&
        config_.robust_scale_factor > 0.0 && std::isfinite(config_.robust_scale_factor))) {
    throw std::invalid_argument("Invalid big rune LM/IRLS configuration");
  }
  config_.outer_max_iterations = std::max(config_.outer_max_iterations, 1);
  config_.irls_iterations = std::max(config_.irls_iterations, 2);
  config_.jump_min_samples = std::max(config_.jump_min_samples, 2);
  config_.prepare_time = std::max(config_.prepare_time, config_.max_time_interval);
  config_.jump_window_seconds = std::max(config_.jump_window_seconds, 0.0);
  config_.jump_min_span_seconds =
    std::clamp(config_.jump_min_span_seconds, 0.0, config_.jump_window_seconds);
  config_.jump_reset_ratio = std::clamp(config_.jump_reset_ratio, 0.0, 1.0);
}

void BigRuneMotionEstimator::clear_model_state(bool clear_candidates, bool clear_direction)
{
  if (clear_candidates) candidate_frames_.clear();
  tracked_targets_.clear();
  jump_samples_.clear();
  model_valid_ = false;
  model_ = {};
  if (clear_direction) rotation_direction_ = 0;
}

void BigRuneMotionEstimator::reset() { clear_model_state(true, true); }

int BigRuneMotionEstimator::vote_rotation_direction() const
{
  int positive = 0;
  int negative = 0;
  const auto vote = [&](double delta) {
    if (std::abs(delta) > config_.target_switch_threshold) return;
    positive += delta > 0.0;
    negative += delta < 0.0;
  };

  for (std::size_t frame = 1; frame < candidate_frames_.size(); ++frame) {
    const auto & previous = candidate_frames_[frame - 1];
    const auto & current = candidate_frames_[frame];
    const double dt = tools::delta_time(
      current.front().capture_timestamp, previous.front().capture_timestamp);
    if (dt <= 0.0 || dt > config_.rotation_vote_max_interval) continue;

    if (previous.size() == 2 && current.size() == 2) {
      const double delta_00 = rune_phase_difference(current[0].phase, previous[0].phase);
      const double delta_11 = rune_phase_difference(current[1].phase, previous[1].phase);
      const double delta_01 = rune_phase_difference(current[0].phase, previous[1].phase);
      const double delta_10 = rune_phase_difference(current[1].phase, previous[0].phase);
      if (std::abs(delta_00) + std::abs(delta_11) <=
          std::abs(delta_01) + std::abs(delta_10)) {
        vote(delta_00);
        vote(delta_11);
      } else {
        vote(delta_01);
        vote(delta_10);
      }
      continue;
    }

    double best_delta = 0.0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (const auto & current_target : current) {
      for (const auto & previous_target : previous) {
        const double delta = rune_phase_difference(current_target.phase, previous_target.phase);
        if (std::abs(delta) < best_cost) {
          best_cost = std::abs(delta);
          best_delta = delta;
        }
      }
    }
    vote(best_delta);
  }
  return positive == negative ? 0 : (positive > negative ? 1 : -1);
}

std::size_t BigRuneMotionEstimator::choose_target( // 选择沿旋转方向位于前方的那个作为跟踪和击打候选
  const std::vector<RuneCandidateTarget> & candidates) const
{
  if (candidates.size() < 2) return 0;
  const double delta = rune_phase_difference(candidates[0].phase, candidates[1].phase);
  if (rotation_direction_ > 0) return delta > 0.0 ? 0U : 1U;
  return delta > 0.0 ? 1U : 0U;
}

double BigRuneMotionEstimator::expected_phase(RuneTimestamp timestamp) const // 根据预测运动方程计算出dt后所旋转到达的角度
{
  if (model_valid_) {
    const double t = tools::delta_time(timestamp, model_.reference_timestamp);
    return model_.phase_cos_coefficient * std::cos(model_.speed_angular_frequency * t) +
           model_.phase_sin_coefficient * std::sin(model_.speed_angular_frequency * t) +
           model_.phase_linear_velocity * t + model_.phase_constant_offset_radians;
  }

  const TimedPhase & latest = tracked_targets_.back();
  const double dt = tools::delta_time(timestamp, latest.target.capture_timestamp);
  return latest.continuous_phase +
         rotation_direction_ * config_.expected_linear_velocity * std::max(dt, 0.0);
}

void BigRuneMotionEstimator::compensate_target_switch(
  const RuneCandidateTarget & candidate, double predicted_phase)
{
  const double observed_jump = rune_phase_difference(candidate.phase, predicted_phase);
  int best_switch_count = 0;
  double best_residual = std::numeric_limits<double>::infinity();
  for (int count = -2; count <= 2; ++count) { // 找到跳了几块靶
    if (count == 0) continue;
    const double residual = std::abs(observed_jump - count * kSectorPhase);
    if (residual < best_residual) {
      best_residual = residual;
      best_switch_count = count;
    }
  }

  const double phase_compensation = best_switch_count * kSectorPhase;
  for (auto & target : tracked_targets_) // 根据跳了几块靶补偿所有tracked的目标
    target.continuous_phase += phase_compensation;

  TimedPhase switched;
  switched.target = candidate;
  switched.switch_count = best_switch_count;
  switched.continuous_phase = tracked_targets_.back().continuous_phase + // 处理相位跨界问题
                              rune_phase_difference(
                                candidate.phase, tracked_targets_.back().continuous_phase);
  tracked_targets_.push_back(std::move(switched)); // 重新构建发生跳跃的靶

  // Keep the fitting values bounded without changing their represented angle.
  const double first_phase = tracked_targets_.front().continuous_phase;
  const double bounded_first_phase = normalize_rune_phase(first_phase);
  const double overflow_compensation = bounded_first_phase - first_phase;
  if (overflow_compensation != 0.0) {
    for (auto & target : tracked_targets_)
      target.continuous_phase += overflow_compensation; // 对所有进行相位补偿（+or-2*k*PI）
  }
}

bool BigRuneMotionEstimator::append_tracked_target( // 将候选帧中的目标添加到已跟踪的目标中
  const std::vector<RuneCandidateTarget> & candidates)
{
  if (candidates.empty()) return false;
  const RuneCandidateTarget & selected = candidates[choose_target(candidates)];
  if (tracked_targets_.empty()) {
    tracked_targets_.push_back({selected, selected.phase, 0});
    return false;
  }

  const double predicted = expected_phase(selected.capture_timestamp);
  const bool is_jump =
    std::abs(rune_phase_difference(predicted, selected.phase)) >
    config_.target_switch_threshold;
  if (is_jump) {
    compensate_target_switch(selected, predicted); // 发生跳跃则补偿所有已跟踪的靶
  } else {
    const TimedPhase & previous = tracked_targets_.back();
    tracked_targets_.push_back(
      {selected,
       previous.continuous_phase +
         rune_phase_difference(selected.phase, previous.continuous_phase),
       0});
  }
  return is_jump;
}

void BigRuneMotionEstimator::rebuild_track()
{
  tracked_targets_.clear();
  const bool previous_model_valid = model_valid_;
  model_valid_ = false;
  for (const auto & frame : candidate_frames_)
    append_tracked_target(frame);
  model_valid_ = previous_model_valid;
}

void BigRuneMotionEstimator::prune_windows() // 删去对应时间过旧的数据
{
  if (!candidate_frames_.empty()) { //针对原始候选帧
    const RuneTimestamp newest = candidate_frames_.back().front().capture_timestamp;
    while (!candidate_frames_.empty() && 
           tools::delta_time(newest, candidate_frames_.front().front().capture_timestamp) >
             config_.window_seconds) {
      candidate_frames_.pop_front();
    }
  }
  if (!tracked_targets_.empty()) { //针对已跟踪的目标
    const RuneTimestamp newest = tracked_targets_.back().target.capture_timestamp;
    while (!tracked_targets_.empty() &&
           tools::delta_time(newest, tracked_targets_.front().target.capture_timestamp) >
             config_.window_seconds) {
      tracked_targets_.pop_front();
    }
  }
}

bool BigRuneMotionEstimator::too_many_abnormal_jumps()
{
  if (jump_samples_.empty()) return false;
  const RuneTimestamp newest = jump_samples_.back().timestamp;
  while (!jump_samples_.empty() && // 删除距离最新记录过久的旧样本
         tools::delta_time(newest, jump_samples_.front().timestamp) >
           config_.jump_window_seconds) {
    jump_samples_.pop_front();
  }
  if (static_cast<int>(jump_samples_.size()) < config_.jump_min_samples)
    return false;
  if (tools::delta_time(jump_samples_.back().timestamp, jump_samples_.front().timestamp) <
      config_.jump_min_span_seconds) {
    return false;
  }

  const int jump_count = static_cast<int>(std::count_if(
    jump_samples_.begin(), jump_samples_.end(),
    [](const JumpSample & sample) { return sample.is_jump; }));
  return static_cast<double>(jump_count) / jump_samples_.size() > config_.jump_reset_ratio; // 如果跳跃样本占比过大则认为运动模型不可靠
}

bool BigRuneMotionEstimator::fit_motion_model()
{
  //拟合大符的运动方程phase = A*cos(ωt) + B*sin(ωt) + b*t + C或-phase = A*cos(ωt) + B*sin(ωt) + b*t + C
  //拟合方法是LM + 迭代加权最小二乘算法
  //外部循环迭代ω
  //内部迭代A,B,b,C
  //外层用LM算法迭代ω，内层用迭代加权最小二乘拟合A,B,b,C
  //1. 初始化ω,A,B,b,C
  //2. 按照时间为参数分配权重
  //3. 内层循环第一次迭代。仅有时间加权。迭代结果与测量值作差，得到距离权重。
  //4. 根据距离权重迭代和时间权重进行迭代，迭代后重新计算距离权重,继续迭代直至达到内层的终止条件
  //5. 外层采用lm算法进行迭代

  if (static_cast<int>(tracked_targets_.size()) < config_.min_data_size)
    return false;

  // 取中值时间戳代替平均时间戳；本项目 delta_time 的单位已经是秒。
  const RuneTimestamp first_timestamp = tracked_targets_.front().target.capture_timestamp;
  const RuneTimestamp reference =
    tracked_targets_[tracked_targets_.size() / 2].target.capture_timestamp;
  const double duration = tools::delta_time(
    tracked_targets_.back().target.capture_timestamp, first_timestamp);
  if (!(duration > 0.0)) return false;
  const Eigen::Index size = static_cast<Eigen::Index>(tracked_targets_.size());
  Eigen::VectorXd times(size), observation(size), time_weights(size);
  for (Eigen::Index i = 0; i < size; ++i) {
    times[i] = tools::delta_time(tracked_targets_[i].target.capture_timestamp, reference);
    observation[i] = tracked_targets_[i].continuous_phase;
    // 线性分配时间权重，越靠近最新的样本权重越大。
    const double age = tools::delta_time(
      tracked_targets_[i].target.capture_timestamp, first_timestamp);
    time_weights[i] = config_.lowest_time_weight +
      (config_.highest_time_weight - config_.lowest_time_weight) * age / duration;
  }

  // 固定 ω，用 Cauchy IRLS 求解线性参数 A、B、b、C。
  // 每次评估独立初始化；失败不能沿用上一次评估的有效标志或参数。
  const auto eval_omega = [&](double omega, Eigen::Vector4d &theta, double &cost) {
    Eigen::MatrixXd X(size, 4);
    for (Eigen::Index i = 0; i < size; ++i)
      X.row(i) << std::cos(omega * times[i]), std::sin(omega * times[i]), times[i], 1.0;
    Eigen::VectorXd residual_weights = Eigen::VectorXd::Ones(size);
    Eigen::Vector4d previous_theta = Eigen::Vector4d::Zero();
    double previous_cost = std::numeric_limits<double>::infinity();
    for (int iteration = 0; iteration < config_.irls_iterations; ++iteration) {
      // 总权重为时间权重和残差权重的逐元素乘积。
      const Eigen::VectorXd weights = time_weights.cwiseProduct(residual_weights);
      // 加权正规方程：(X^T W X + ridge I) theta = X^T W y。
      Eigen::Matrix4d H = X.transpose() * weights.asDiagonal() * X;
      H.diagonal().array() += 1e-6;  // 岭正则抑制短时间窗口下的病态问题。
      const Eigen::Vector4d g = X.transpose() * weights.asDiagonal() * observation; // 加权观测向量
      if (!H.allFinite() || !g.allFinite()) return false;
      const Eigen::LDLT<Eigen::Matrix4d> ldlt(H);
      if (ldlt.info() != Eigen::Success) return false;
      const Eigen::Vector4d next_theta = ldlt.solve(g); // 求解正规方程
      if (!next_theta.allFinite()) return false;
      const Eigen::VectorXd residual = observation - X * next_theta; // 计算残差
      const double next_cost = (weights.array() * residual.array().square()).sum(); // 计算加权残差平方和
      if (!std::isfinite(next_cost)) return false;

      // RP 使用时间加权 RMS 确定 Cauchy 尺度
      const double rms = std::sqrt(std::max(
        1e-12, (time_weights.array() * residual.array().square()).sum() / size));
      const double scale = std::max(1e-6, config_.robust_scale_factor * rms);
      residual_weights = (1.0 / (1.0 + (residual.array() / scale).square())).matrix();
      // 第一次求解不判断收敛；后续与上一轮参数或代价比较。
      if (iteration > 0 &&
          ((previous_theta - next_theta).norm() < config_.inner_tolerance ||
           std::abs(previous_cost - next_cost) < config_.inner_tolerance)) {
        theta = next_theta;
        cost = next_cost;
        return true;
      }
      previous_theta = next_theta;
      previous_cost = next_cost;
    }
    return false;  // 达到内层迭代上限仍未收敛，本次评估无效。
  };

  const auto clamp_omega = [&](double omega) {
    return std::clamp(omega, config_.omega_lower_bound, config_.omega_upper_bound);
  };
  // 延续上一帧频率；线性参数在新的时间原点下重新求解，无需搬用旧系数。
  double omega = clamp_omega(model_valid_ ? model_.speed_angular_frequency : config_.omega_init);
  double best_omega = omega;
  double best_cost = std::numeric_limits<double>::infinity();
  Eigen::Vector4d best_coefficients;
  if (!eval_omega(omega, best_coefficients, best_cost)) return false;

  double lambda = std::clamp(config_.lambda_init, config_.lambda_min, config_.lambda_max);
  for (int iteration = 0; iteration < config_.outer_max_iterations; ++iteration) {
    const double eps = std::max(config_.diff_eps_min, config_.diff_eps_rel * std::abs(omega));
    // 边界附近向区间内部取三个点，避免截断中心差分后错误估计曲率。
    const double center = std::clamp(
      omega, config_.omega_lower_bound + std::min(eps, (config_.omega_upper_bound - config_.omega_lower_bound) / 2.0),
      config_.omega_upper_bound - std::min(eps, (config_.omega_upper_bound - config_.omega_lower_bound) / 2.0));
    const double plus = clamp_omega(center + eps);
    const double minus = clamp_omega(center - eps);
    if (plus == minus) break;
    Eigen::Vector4d theta;
    double cost_plus, cost_minus, cost_center;
    if (!eval_omega(plus, theta, cost_plus) || !eval_omega(minus, theta, cost_minus) ||
        !eval_omega(center, theta, cost_center)) {
      lambda = std::min(config_.lambda_max, lambda * 2.0); // 增大阻尼，避免在曲率不可靠时过度迭代
      continue;
    }
    const double half_span = 0.5 * (plus - minus);
    const double curvature = (cost_plus - 2.0 * cost_center + cost_minus) / (half_span * half_span); // 计算二阶导数近似曲率
    const double gradient = (cost_plus - cost_minus) / (plus - minus) + curvature * (omega - center); // 计算一阶导数近似梯度
    if (!std::isfinite(gradient) || !std::isfinite(curvature)) return false;
    const double step = std::clamp( // LM算法更新步长，步长 = -梯度 / (曲率 + 阻尼)，并限制最大步长
      -gradient / (std::abs(curvature) + lambda), -config_.max_omega_step, config_.max_omega_step);
    const double candidate = clamp_omega(omega + step);
    if (std::abs(candidate - omega) < config_.outer_tolerance) break;
    double cost;
    // 只接受降低代价的步长；成功减小阻尼，失败增大阻尼。
    if (eval_omega(candidate, theta, cost) && cost < best_cost) {
      omega = candidate;
      best_omega = candidate;
      best_coefficients = theta;
      best_cost = cost;
      lambda = std::max(config_.lambda_min, lambda * 0.5);
    } else {
      lambda = std::min(config_.lambda_max, lambda * 2.0);
    }
  }

  model_.reference_timestamp = reference;
  model_.phase_cos_coefficient = best_coefficients[0];
  model_.phase_sin_coefficient = best_coefficients[1];
  model_.phase_linear_velocity = best_coefficients[2];
  model_.phase_constant_offset_radians = best_coefficients[3];
  model_.speed_angular_frequency = best_omega;
  model_.speed_amplitude = best_omega * std::hypot(best_coefficients[0], best_coefficients[1]);
  model_.speed_phase_shift = std::atan2(best_coefficients[1], -best_coefficients[0]);
  if (model_.speed_phase_shift < 0.0) model_.speed_phase_shift += 2.0 * M_PI;
  return true;
}

RuneTarget BigRuneMotionEstimator::make_target(int inactive_target_num) const
{
  const TimedPhase & tracked = tracked_targets_.back();
  RuneTarget target;
  target.rune_center = tracked.target.rune_center;
  target.armor_module_center = tracked.target.armor_module_center;
  target.start_vector = tracked.target.start_vector;
  target.rune_plane_world_normal = tracked.target.rune_plane_world_normal;
  target.phase = normalize_rune_phase(tracked.target.phase);
  target.angular_velocity = 0.0;
  target.big_rune_motion_model = model_;
  target.capture_timestamp = tracked.target.capture_timestamp;
  target.inactive_target_num = inactive_target_num;
  target.is_big_rune = true;
  return target;
}

std::optional<RuneTarget> BigRuneMotionEstimator::estimate(
  const std::vector<RuneCandidateTarget> & candidates, int inactive_target_num)
{
  if (candidates.empty() || candidates.size() > 2) return std::nullopt;

  if (!candidate_frames_.empty()) { // 时间过久重置与dt过大重置运动状态
    const double dt = tools::delta_time(
      candidates.front().capture_timestamp,
      candidate_frames_.back().front().capture_timestamp);
    if (dt <= 0.0) return std::nullopt;
    if (dt > config_.prepare_time) {
      reset();
    } else if (dt > config_.max_time_interval) {
      clear_model_state(true, false);
    }
  }

  candidate_frames_.push_back(candidates);
  prune_windows(); // 维护大符运动估计的滑动时间窗口，删除过旧的数据

  if (rotation_direction_ == 0) {
    if (static_cast<int>(candidate_frames_.size()) < config_.min_data_size)
      return std::nullopt;
    rotation_direction_ = vote_rotation_direction();
    if (rotation_direction_ == 0) return std::nullopt;
    tools::logger()->info(
      "[BigRuneMotionEstimator] rotation direction: {}",
      rotation_direction_ > 0 ? "positive" : "negative");
    rebuild_track();
  } else if (tracked_targets_.empty()) {
    rebuild_track(); // 相当于初始化
  } else {
    const bool is_jump = append_tracked_target(candidates);
    jump_samples_.push_back({is_jump, candidates.front().capture_timestamp});
    prune_windows();
    if (too_many_abnormal_jumps()) {
      // Preserve the raw observation window and known direction. The next frame rebuilds a
      // compensated continuous track from those observations before fitting again.
      clear_model_state(false, false);
      tools::logger()->warn(
        "[BigRuneMotionEstimator] reset model after frequent target phase jumps");
      return std::nullopt;
    }
  }

  model_valid_ = fit_motion_model();
  if (!model_valid_ || tracked_targets_.empty()) return std::nullopt;
  return make_target(inactive_target_num);
}
}  // namespace auto_buff
