#ifndef AUTO_BUFF__RUNE_POSE_OPTIMIZER_HPP
#define AUTO_BUFF__RUNE_POSE_OPTIMIZER_HPP

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <optional>
#include <string>
#include <vector>

#include "rune_camera.hpp"
#include "rune_observation.hpp"

namespace auto_buff
{
class RunePoseOptimizer
{
public:
  struct Result
  {
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    std::vector<int> inactive_locations;
  };

  explicit RunePoseOptimizer(const std::string & config_path);

  std::optional<Result> optimize(
    const RefinedRuneObservation & observation, const CameraPose & camera_pose,
    const RuneCamera & camera) const;

private:
  int sample_stride_ = 2;
  double distance_transform_roi_scale_ = 1.3;
  double anchor_reproj_weight_ = 7.0;
  double max_chamfer_residual_ = 50.0;
  double normal_constraint_weight_total_ = 500.0;
  double half_interval_ = 0.5235987756;
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__RUNE_POSE_OPTIMIZER_HPP
