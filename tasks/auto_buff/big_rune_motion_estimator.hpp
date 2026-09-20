#ifndef AUTO_BUFF__BIG_RUNE_MOTION_ESTIMATOR_HPP
#define AUTO_BUFF__BIG_RUNE_MOTION_ESTIMATOR_HPP

#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "rune_tracker.hpp"

namespace auto_buff
{
class BigRuneMotionEstimator
{
public:
  explicit BigRuneMotionEstimator(const std::string & config_path);

  std::optional<RuneTarget> estimate(
    const std::vector<RuneCandidateTarget> & candidates, int inactive_target_num);
  void reset();

private:
  struct Config
  {
    double max_time_interval = 3.0;
    double prepare_time = 180.0;
    double window_seconds = 8.0;
    int min_data_size = 100;
    double rotation_vote_max_interval = 0.15;
    double target_switch_threshold = 0.77;
    double expected_linear_velocity = 1.1775;
    double omega_lower_bound = 1.884;
    double omega_upper_bound = 2.0;
    double omega_init = 1.942;
    int outer_max_iterations = 20;
    double outer_tolerance = 1e-5;
    double lambda_init = 1e-2;
    double lambda_min = 1e-6;
    double lambda_max = 1e6;
    double max_omega_step = 0.05;
    double diff_eps_min = 1e-4;
    double diff_eps_rel = 1e-3;
    double highest_time_weight = 1.0;
    double lowest_time_weight = 0.2;
    int irls_iterations = 40;
    double inner_tolerance = 1e-2;
    double robust_scale_factor = 2.5;
    double jump_window_seconds = 1.0;
    double jump_min_span_seconds = 0.7;
    int jump_min_samples = 10;
    double jump_reset_ratio = 0.3;
  } config_;

  struct TimedPhase
  {
    RuneCandidateTarget target;
    double continuous_phase = 0.0;
    int switch_count = 0;
  };

  struct JumpSample
  {
    bool is_jump = false;
    RuneTimestamp timestamp{};
  };

  std::deque<std::vector<RuneCandidateTarget>> candidate_frames_;
  std::deque<TimedPhase> tracked_targets_;
  std::deque<JumpSample> jump_samples_;
  int rotation_direction_ = 0;
  bool model_valid_ = false;
  BigRuneMotionModel model_;

  int vote_rotation_direction() const;
  std::size_t choose_target(const std::vector<RuneCandidateTarget> & candidates) const;
  bool append_tracked_target(const std::vector<RuneCandidateTarget> & candidates);
  void rebuild_track();
  double expected_phase(RuneTimestamp timestamp) const;
  void compensate_target_switch(const RuneCandidateTarget & candidate, double predicted_phase);
  bool too_many_abnormal_jumps();
  bool fit_motion_model();
  void clear_model_state(bool clear_candidates, bool clear_direction);
  void prune_windows();
  RuneTarget make_target(int inactive_target_num) const;
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__BIG_RUNE_MOTION_ESTIMATOR_HPP
