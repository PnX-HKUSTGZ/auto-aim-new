#ifndef AUTO_BUFF__POWER_RUNE_PLANE_HPP
#define AUTO_BUFF__POWER_RUNE_PLANE_HPP

#include <opencv2/opencv.hpp>

#include <optional>
#include <string>
#include <vector>

#include "rune_observation.hpp"
#include "rune_pose_optimizer.hpp"
#include "rune_tracker.hpp"

namespace auto_buff
{
class PowerRunePlane
{
public:
  struct ReconstructedPiece
  {
    InactiveTargets::RunePiece piece;
    std::vector<Eigen::Vector3d> plane_points;
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_world_model = Eigen::Matrix3d::Identity();
  };

  struct Result
  {
    InactiveTargets targets;
    std::vector<ReconstructedPiece> reconstructed;
  };

  explicit PowerRunePlane(const std::string & config_path);

  std::optional<Result> update_power_rune_plane(
    const RefinedRuneObservation & refined_observation, const CameraPose & camera_pose,
    const RuneCamera & camera) const;

  void visualize_power_rune_plane(
    cv::Mat & image, const std::vector<ReconstructedPiece> & reconstructed,
    const InactiveTargets & targets, const CameraPose & camera_pose,
    const RuneCamera & camera) const;

private:
  double rune_radius_ = 0.7;
  bool visualize_ = false;
  RunePoseOptimizer pose_optimizer_;

  std::optional<ReconstructedPiece> reconstruct_inactive_piece(
    const SingleRuneBlade2D & blade, const CameraPose & camera_pose,
    const RuneCamera & camera) const;
  static bool valid_keypoints(const RuneInfo & info);
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__POWER_RUNE_PLANE_HPP
