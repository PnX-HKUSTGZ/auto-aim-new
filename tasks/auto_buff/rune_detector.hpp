#ifndef AUTO_BUFF__RUNE_DETECTOR_HPP
#define AUTO_BUFF__RUNE_DETECTOR_HPP

#include <chrono>
#include <optional>
#include <string>

#include "power_rune_plane.hpp"
#include "rune_camera.hpp"
#include "rune_observation.hpp"
#include "rune_observation_refiner.hpp"
#include "yolo11_buff.hpp"
namespace auto_buff
{
class RuneDetector
{
public:
  RuneDetector(const std::string & config);

  std::optional<InactiveTargets> detect(
    cv::Mat & bgr_img, PowerRuneType type,
    std::chrono::steady_clock::time_point timestamp, const CameraPose & camera_pose,
    const RuneCamera & camera);

private:
  YOLO11_BUFF MODE_;
  RuneObservationConverter observation_converter_;
  RuneObservationRefiner observation_refiner_;
  PowerRunePlane power_rune_plane_;
};
}  // namespace auto_buff
#endif  // AUTO_BUFF__RUNE_DETECTOR_HPP
