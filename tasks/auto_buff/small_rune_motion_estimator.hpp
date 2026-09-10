#ifndef AUTO_BUFF__SMALL_RUNE_MOTION_ESTIMATOR_HPP
#define AUTO_BUFF__SMALL_RUNE_MOTION_ESTIMATOR_HPP

#include <deque>
#include <optional>
#include <string>

#include "rune_tracker.hpp"

namespace auto_buff
{
class SmallRuneMotionEstimator
{
public:
  explicit SmallRuneMotionEstimator(const std::string & config_path);

  std::optional<RuneTarget> estimate(const RuneCandidateTarget & candidate);
  void reset();

private:
  struct Config
  {
    double max_data_interval = 0.1;
    double prepare_time = 180.0;
    int min_size_to_confirm_rotation = 20;
    double target_switch_threshold = 0.87;
    int min_size_to_filter = 2;
    double expected_angular_velocity = 1.0472;
    double angular_tolerance_factor = 3.0;
    double p_init = 1.0;
    double q = 1.0;
    double r = 0.025;
  } config_;

  std::deque<RuneCandidateTarget> candidates_;
  int rotation_direction_ = 0;
  bool filter_initialized_ = false;
  double posterior_phase_ = 0.0;
  double covariance_ = 1.0;
  RuneTimestamp filter_timestamp_{};

  int vote_rotation_direction() const;
  void reset_filter();
  std::optional<RuneTarget> make_target(const RuneCandidateTarget & candidate) const;
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__SMALL_RUNE_MOTION_ESTIMATOR_HPP
