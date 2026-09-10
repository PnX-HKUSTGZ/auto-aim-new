#include "small_rune_motion_estimator.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

#include "tools/logger.hpp"

namespace auto_buff
{
namespace
{
template<typename T>
void read_optional(const YAML::Node & node, const char * key, T & value)
{
  if (node && node[key]) value = node[key].as<T>();
}
}  // namespace

SmallRuneMotionEstimator::SmallRuneMotionEstimator(const std::string & config_path)
{
  const YAML::Node phase = YAML::LoadFile(config_path)["buff_phase"];
  read_optional(phase, "small_max_data_interval", config_.max_data_interval);
  read_optional(phase, "small_prepare_time", config_.prepare_time);
  read_optional(
    phase, "small_min_size_to_confirm_rotation", config_.min_size_to_confirm_rotation);
  read_optional(phase, "small_target_switch_threshold", config_.target_switch_threshold);
  read_optional(phase, "small_min_size_to_filter", config_.min_size_to_filter);
  read_optional(
    phase, "small_expected_angular_velocity", config_.expected_angular_velocity);
  read_optional(
    phase, "small_angular_tolerance_factor", config_.angular_tolerance_factor);
  read_optional(phase, "small_p_init", config_.p_init);
  read_optional(phase, "small_q", config_.q);
  read_optional(phase, "small_r", config_.r);

  config_.min_size_to_confirm_rotation =
    std::max(config_.min_size_to_confirm_rotation, 2);
  config_.min_size_to_filter = std::max(config_.min_size_to_filter, 2);
  config_.prepare_time = std::max(config_.prepare_time, config_.max_data_interval);
}

void SmallRuneMotionEstimator::reset_filter()
{
  candidates_.clear();
  filter_initialized_ = false;
  posterior_phase_ = 0.0;
  covariance_ = config_.p_init;
  filter_timestamp_ = {};
}

void SmallRuneMotionEstimator::reset()
{
  reset_filter();
  rotation_direction_ = 0;
}

int SmallRuneMotionEstimator::vote_rotation_direction() const
{
  int positive = 0;
  int negative = 0;
  for (std::size_t i = 1; i < candidates_.size(); ++i) {
    const double delta = rune_phase_difference(candidates_[i].phase, candidates_[i - 1].phase);
    if (std::abs(delta) > config_.target_switch_threshold) continue;
    positive += delta > 0.0;
    negative += delta < 0.0;
  }
  return positive == negative ? 0 : (positive > negative ? 1 : -1);
}

std::optional<RuneTarget> SmallRuneMotionEstimator::make_target(
  const RuneCandidateTarget & candidate) const
{
  RuneTarget target;
  target.rune_center = candidate.rune_center;
  target.armor_module_center = candidate.armor_module_center;
  target.start_vector = candidate.start_vector;
  target.rune_plane_world_normal = candidate.rune_plane_world_normal;
  target.phase = normalize_rune_phase(posterior_phase_);
  target.angular_velocity =
    rotation_direction_ * config_.expected_angular_velocity;
  target.capture_timestamp = candidate.capture_timestamp;
  target.inactive_target_num = 1;
  target.is_big_rune = false;
  return target;
}

std::optional<RuneTarget> SmallRuneMotionEstimator::estimate(
  const RuneCandidateTarget & candidate)
{
  if (!candidates_.empty()) {
    const double dt = // 时间过久重置与dt过大重置运动状态
      tools::delta_time(candidate.capture_timestamp, candidates_.back().capture_timestamp);
    if (dt <= 0.0) return std::nullopt;
    if (dt > config_.prepare_time) {
      reset();
    } else if (dt > config_.max_data_interval) {
      reset_filter();
    }
  }
  candidates_.push_back(candidate);

  if (rotation_direction_ == 0) {
    if (static_cast<int>(candidates_.size()) < config_.min_size_to_confirm_rotation)
      return std::nullopt;
    rotation_direction_ = vote_rotation_direction();
    if (rotation_direction_ == 0) return std::nullopt;
    tools::logger()->info(
      "[SmallRuneMotionEstimator] rotation direction: {}",
      rotation_direction_ > 0 ? "positive" : "negative");
  }

  while (static_cast<int>(candidates_.size()) > config_.min_size_to_filter) // 暴力约束candidates数量
    candidates_.pop_front();
  if (static_cast<int>(candidates_.size()) < config_.min_size_to_filter)
    return std::nullopt;

  const double innovation_limit = config_.max_data_interval * // 相位观测允许偏离正常运动预测的最大角度阈值
                                  config_.expected_angular_velocity *
                                  config_.angular_tolerance_factor;
  const double latest_observation_delta =
    rune_phase_difference(candidates_.back().phase, candidates_[candidates_.size() - 2].phase);
  if (std::abs(latest_observation_delta) > innovation_limit) {
    // An activated blade can disappear and make the selected inactive blade jump by 2*pi/5.
    // Keep the established direction, but restart the filter around the new observation.
    const RuneCandidateTarget new_root = candidates_.back();
    reset_filter();
    candidates_.push_back(new_root);
    tools::logger()->debug("[SmallRuneMotionEstimator] reset after target phase jump");
    return std::nullopt;
  }

  const double angular_velocity =
    rotation_direction_ * config_.expected_angular_velocity;
  if (!filter_initialized_) { // 初始化滤波器
    posterior_phase_ = candidates_.front().phase;
    covariance_ = config_.p_init;
    filter_timestamp_ = candidates_.front().capture_timestamp;
    filter_initialized_ = true;
  }

  const double dt = tools::delta_time(candidate.capture_timestamp, filter_timestamp_);
  if (dt < 0.0) return std::nullopt;
  // 线性卡尔曼滤波器预测更新和观测更新
  const double prior_phase = posterior_phase_ + angular_velocity * dt; // 预测相位更新
  const double innovation = rune_phase_difference(candidate.phase, prior_phase);
  if (std::abs(innovation) > innovation_limit) {
    const RuneCandidateTarget new_root = candidate;
    reset_filter();
    candidates_.push_back(new_root);
    tools::logger()->debug("[SmallRuneMotionEstimator] reset after prediction phase jump");
    return std::nullopt;
  }
  covariance_ += config_.q * dt * dt; // 预测协方差更新
  const double kalman_gain = covariance_ / (covariance_ + config_.r); // 卡尔曼增益计算
  posterior_phase_ = prior_phase + kalman_gain * innovation; // 后验相位更新
  covariance_ *= 1.0 - kalman_gain; // 后验协方差更新
  // posterior_phase_ now represents the state at this observation time. Advance the
  // filter clock as well, otherwise the next prediction integrates the elapsed time
  // from the initialization frame again and repeatedly over-predicts the phase.
  filter_timestamp_ = candidate.capture_timestamp;
  return make_target(candidate);
}
}  // namespace auto_buff
