#ifndef AUTO_AIM__AUTO_AIM_SCENE_HPP
#define AUTO_AIM__AUTO_AIM_SCENE_HPP

#include <cstdint>
#include <list>
#include <string>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include "target.hpp"

namespace auto_aim
{
class AutoAimVisualizer
{
public:
  explicit AutoAimVisualizer(const std::string & config_path);
  explicit AutoAimVisualizer(double air_resistance);

  // yaw/pitch are the original Planner command angles in radians (muzzle-up
  // pitch is negative). attitude rotates the gimbal frame into the world frame.
  // All ballistic conversion and sampling stays inside this visualization module.
  nlohmann::json make_scene(
    const std::list<Target> & targets, double bullet_speed, double yaw, double pitch,
    const Eigen::Quaterniond & attitude, std::uint64_t timestamp_ns) const;

  // Register the gimbal pose in Foxglove's frame tree even when no target exists.
  nlohmann::json make_gimbal_transform(
    const Eigen::Quaterniond & attitude, std::uint64_t timestamp_ns) const;

private:
  double air_resistance_ = 0.1;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__AUTO_AIM_SCENE_HPP
