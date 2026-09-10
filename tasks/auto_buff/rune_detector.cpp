#include "rune_detector.hpp"

#include <utility>

namespace auto_buff
{
RuneDetector::RuneDetector(const std::string & config)
: MODE_(config),
  observation_converter_(config),
  observation_refiner_(config),
  power_rune_plane_(config)
{
}

std::optional<InactiveTargets> RuneDetector::detect(
  cv::Mat & bgr_img, PowerRuneType type, std::chrono::steady_clock::time_point timestamp,
  const CameraPose & camera_pose, const RuneCamera & camera)
{
  const std::vector<YOLO11_BUFF::Object> detections = MODE_.get_multicandidateboxes(bgr_img);
  if (detections.empty()) return std::nullopt;

  // 构造符的观测数据
  RuneObservation observation = observation_converter_.convert2rune_observation(
    bgr_img, detections, type, timestamp);
  if (observation.rune_infos.empty()) return std::nullopt;

  // refine must run before any visualization modifies the source image.
  RefinedRuneObservation refined = observation_refiner_.refine(observation);
  if (refined.rune_blades.empty()) return std::nullopt;

  auto plane_result = power_rune_plane_.update_power_rune_plane(refined, camera_pose, camera);
  if (!plane_result) return std::nullopt;

  observation_converter_.visualize_rune_observation(bgr_img, observation);
  observation_refiner_.visualize_refined_observation(bgr_img, refined);
  power_rune_plane_.visualize_power_rune_plane(
    bgr_img, plane_result->reconstructed, plane_result->targets, camera_pose, camera);
  return std::move(plane_result->targets);
}

}  // namespace auto_buff
