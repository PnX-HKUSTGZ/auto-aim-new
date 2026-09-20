#ifndef AUTO_BUFF__PHASE_MOTION_ESTIMATOR_HPP
#define AUTO_BUFF__PHASE_MOTION_ESTIMATOR_HPP

#include <optional>
#include <string>
#include <vector>

#include "big_rune_motion_estimator.hpp"
#include "small_rune_motion_estimator.hpp"

namespace auto_buff
{
class PhaseMotionEstimator
{
public:
  explicit PhaseMotionEstimator(const std::string & config_path);

  std::optional<RuneTarget> estimate_phase_motion(const InactiveTargets & inactive_targets);
  void reset();

private:
  SmallRuneMotionEstimator small_estimator_;
  BigRuneMotionEstimator big_estimator_;
  std::optional<bool> last_mode_was_big_;

  static std::vector<RuneCandidateTarget> generate_candidate_targets(
    const InactiveTargets & inactive_targets);
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__PHASE_MOTION_ESTIMATOR_HPP
