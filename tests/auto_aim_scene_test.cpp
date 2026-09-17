#include <cmath>
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include "foxglove_scene_schema.hpp"
#include "tasks/auto_aim/auto_aim_scene.hpp"
#include "tools/foxglove_server.hpp"

namespace
{
using Json = nlohmann::json;
using Vec = Eigen::Vector3d;

void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

Vec xyz(const Json & point)
{
  return {
    point.at("x").get<double>(), point.at("y").get<double>(),
    point.at("z").get<double>()};
}

const Json & entity(const Json & scene, const std::string & id)
{
  for (const auto & item : scene.at("entities")) {
    if (item.at("id") == id) return item;
  }
  throw std::runtime_error("Missing entity: " + id);
}

bool has_entity(const Json & scene, const std::string & id)
{
  for (const auto & item : scene.at("entities")) {
    if (item.at("id") == id) return true;
  }
  return false;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    constexpr std::uint64_t stamp = 1700000000123456789ULL;
    auto_aim::Target target(2.2, 0.0, 0.2, 0.0);
    target.name = auto_aim::ArmorName::one;
    target.armor_type = auto_aim::ArmorType::small;
    target.jumped = false;
    target.last_id = {0};
    const std::list<auto_aim::Target> targets{target};

    const auto_aim::AutoAimVisualizer no_drag_visualizer(0.0);
    // 命令 yaw 以云台 +Y 为零方向，-90° 命令对应数学方位角 0（世界 +X）。
    const auto no_drag = no_drag_visualizer.make_scene(
      targets, 20.0, -M_PI / 2, 0.0, Eigen::Quaterniond::Identity(), stamp);
    const auto & no_drag_trajectory = entity(no_drag, "ballistic_trajectory");
    const auto & no_drag_points = no_drag_trajectory.at("lines").front().at("points");
    require(no_drag_points.size() == 81, "Trajectory sample count");
    require(xyz(no_drag_points.front()).norm() < 1e-12, "Trajectory starts at the muzzle");

    const Vec no_drag_endpoint = xyz(
      no_drag_trajectory.at("spheres").front().at("pose").at("position"));
    require(std::abs(no_drag_endpoint.x() - 2.0) < 1e-12, "No-drag horizontal distance");
    require(std::abs(no_drag_endpoint.y()) < 1e-12, "Gimbal command -90 deg points along world X");
    require(std::abs(no_drag_endpoint.z() + 0.049) < 1e-12, "No-drag gravity drop");

    const Json & armor_cube =
      entity(no_drag, "auto_aim_target_0").at("cubes").front();
    const Vec armor_size = xyz(armor_cube.at("size"));
    require(std::abs(armor_size.x() - 0.018) < 1e-12, "Armor is a thin cuboid");
    const Json & armor_rotation = armor_cube.at("pose").at("orientation");
    const Eigen::Quaterniond armor_q(
      armor_rotation.at("w").get<double>(), armor_rotation.at("x").get<double>(),
      armor_rotation.at("y").get<double>(), armor_rotation.at("z").get<double>());
    constexpr double armor_pitch = 22.5 * 3.14159265358979323846 / 180.0;
    const Vec expected_normal(std::cos(armor_pitch), 0.0, -std::sin(armor_pitch));
    require((armor_q * Vec::UnitX() - expected_normal).norm() < 1e-12, "Armor pitch is 22.5 degrees");
    require((armor_q * Vec::UnitY() - Vec::UnitY()).norm() < 1e-12, "Armor roll is zero");

    constexpr double speed = 20.0;
    // 命令 yaw 以云台 +Y 为零方向，-90° 命令对应数学方位角 0（世界 +X）。
    constexpr double yaw = -M_PI / 2;
    constexpr double pitch = 0.1;
    constexpr double distance = 3.0;
    constexpr double resistance = 0.1;
    auto_aim::Target distant_target(3.2, 0.0, 0.2, 0.0);
    distant_target.name = auto_aim::ArmorName::one;
    distant_target.armor_type = auto_aim::ArmorType::small;
    distant_target.jumped = false;
    distant_target.last_id = {0};
    const auto_aim::AutoAimVisualizer drag_visualizer(resistance);
    const Eigen::Quaterniond attitude(Eigen::AngleAxisd(0.5, Vec::UnitZ()));
    const auto with_drag = drag_visualizer.make_scene(
      {distant_target}, speed, yaw, pitch, attitude, stamp);
    const Vec drag_endpoint = xyz(
      entity(with_drag, "ballistic_trajectory")
        .at("spheres").front().at("pose").at("position"));
    const double flight_time =
      std::expm1(resistance * distance) /
      (resistance * speed * std::cos(pitch));
    const double expected_height =
      (speed * std::sin(pitch) + 9.8 / resistance) *
        (-std::expm1(-resistance * flight_time)) / resistance -
      9.8 * flight_time / resistance;
    require(std::abs(drag_endpoint.x() - distance) < 1e-12, "Drag-model horizontal distance");
    require(std::abs(drag_endpoint.y()) < 1e-12, "Gimbal command -90 deg has no world Y travel");
    require(std::abs(drag_endpoint.z() - expected_height) < 1e-12, "Drag-model height");
    const Json & attitude_entity = entity(with_drag, "gimbal_attitude");
    require(attitude_entity.at("arrows").size() == 3, "Attitude draws three gimbal axes");
    require(attitude_entity.at("frame_id") == "gimbal", "Attitude is attached to gimbal frame");
    require(attitude_entity.at("frame_locked"), "Attitude follows the latest gimbal transform");
    const Json transform = drag_visualizer.make_gimbal_transform(attitude, stamp);
    require(transform.at("parent_frame_id") == "world", "Transform parent frame");
    require(transform.at("child_frame_id") == "gimbal", "Transform child frame");
    require(transform.at("timestamp").at("nsec") == 123456789, "Transform timestamp precision");
    const Eigen::Quaterniond transform_q(
      transform.at("rotation").at("w").get<double>(),
      transform.at("rotation").at("x").get<double>(),
      transform.at("rotation").at("y").get<double>(),
      transform.at("rotation").at("z").get<double>());
    require(
      (transform_q * Vec::UnitX() - attitude * Vec::UnitX()).norm() < 1e-12,
      "Gimbal transform preserves the supplied dynamic attitude");
    const Json & camera_entity = entity(with_drag, "camera_frame");
    require(camera_entity.at("frame_id") == "camera", "Camera axes use camera frame");
    require(camera_entity.at("frame_locked"), "Camera axes follow the frame tree");
    const Json camera_transform = drag_visualizer.make_camera_transform(stamp);
    require(camera_transform.at("parent_frame_id") == "gimbal", "Camera transform parent");
    require(camera_transform.at("child_frame_id") == "camera", "Camera transform child");

    auto_aim::Armor observed(
      0, 1.0f, cv::Rect(0, 0, 20, 10), {{0, 0}, {20, 0}, {20, 10}, {0, 10}});
    observed.name = auto_aim::ArmorName::one;
    observed.type = auto_aim::ArmorType::small;
    observed.xyz_in_world = Vec(2.0, -1.0, 0.4);
    observed.ypr_in_world = Vec(0.3, -0.2, 0.1);
    auto_aim::Armor second = observed;
    second.xyz_in_world = Vec(3.0, 1.0, 0.6);
    auto_aim::Armor rejected = observed;
    rejected.name = auto_aim::ArmorName::not_armor;
    const auto observations = drag_visualizer.make_scene(
      {}, speed, yaw, pitch, attitude, stamp, {observed, second, rejected});
    const auto & observation = entity(observations, "observed_armor_0");
    require(observation.at("frame_id") == "world", "Observation uses world frame");
    const auto & observed_pose = observation.at("cubes").front().at("pose");
    require(
      (xyz(observed_pose.at("position")) - observed.xyz_in_world).norm() < 1e-12,
      "World observation must not be rotated again by the gimbal attitude");
    const auto & observed_rotation = observed_pose.at("orientation");
    const Eigen::Quaterniond observed_q(
      observed_rotation.at("w").get<double>(), observed_rotation.at("x").get<double>(),
      observed_rotation.at("y").get<double>(), observed_rotation.at("z").get<double>());
    const Vec observed_normal(
      std::cos(0.3) * std::cos(-0.2), std::sin(0.3) * std::cos(-0.2), -std::sin(-0.2));
    require(
      (observed_q * Vec::UnitX() - observed_normal).norm() < 1e-12,
      "Observation preserves solved world orientation");
    require(has_entity(observations, "observed_armor_1"), "Draw every solved observation without a target");
    require(!has_entity(observations, "observed_armor_2"), "Skip rejected observations");

    const double invalid = std::numeric_limits<double>::quiet_NaN();
    const auto unavailable = drag_visualizer.make_scene(
      targets, invalid, invalid, invalid, Eigen::Quaterniond::Identity(), stamp);
    require(
      !has_entity(unavailable, "ballistic_trajectory"),
      "No plan hides the ballistic trajectory");
    require(unavailable.at("deletions").front().at("type") == 1, "Snapshot clears stale entities");
    if (argc == 3 && std::string(argv[1]) == "--serve") {
      tools::FoxgloveServer server(
        "127.0.0.1", static_cast<std::uint16_t>(std::stoi(argv[2])),
        {{1, "/auto_aim/targets", "foxglove.SceneUpdate", tools::kFoxgloveSceneSchema},
         {2, "/auto_aim/transforms", "foxglove.FrameTransform",
          tools::kFoxgloveTransformSchema},
         {3, "/auto_aim/camera_transform", "foxglove.FrameTransform",
          tools::kFoxgloveTransformSchema}});
      const auto transform = drag_visualizer.make_gimbal_transform(attitude, stamp);
      const auto camera_transform = drag_visualizer.make_camera_transform(stamp);
      for (int i = 0; i < 200; ++i) {
        server.publish(
          stamp,
          {{2, transform.dump()}, {3, camera_transform.dump()}, {1, with_drag.dump()}});
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
      }
    }
    std::cout << "Auto-aim ballistic scene checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
