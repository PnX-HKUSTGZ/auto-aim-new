#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "foxglove_scene_schema.hpp"
#include "tasks/auto_buff/rune_scene.hpp"
#include "tools/foxglove_server.hpp"

namespace
{
using Json = nlohmann::json;
using Vec = Eigen::Vector3d;
void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}
Vec xyz(const Json & p) { return Vec(p.at("x"), p.at("y"), p.at("z")); }
const Json & entity(const Json & scene, const std::string & id)
{
  for (const auto & e : scene.at("entities")) if (e.at("id") == id) return e;
  throw std::runtime_error("Missing entity: " + id);
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    auto_buff::InactiveTargets observed;
    const Vec center(3, 1, 0.5);
    const Vec normal = Vec(1, 2, 3).normalized();
    const Vec radial = normal.unitOrthogonal();
    observed.power_rune_plane = Eigen::Hyperplane<double, 3>(normal, center);
    observed.rune_pieces.push_back({center, center + 0.7 * radial});
    auto_buff::RuneTarget target;
    target.rune_center = center;
    target.armor_module_center = observed.rune_pieces.front().armor_center;
    target.start_vector = radial;
    target.rune_plane_world_normal = normal;
    target.angular_velocity = 1.0;
    const auto stamp = std::uint64_t(1700000000123456789ULL);
    auto_buff::CameraPose camera;
    camera.t_world_camera = Vec(0.1, -0.2, 0.3);
    camera.R_world_camera = Eigen::AngleAxisd(0.4, Vec::UnitY()).toRotationMatrix();
    const auto transform = auto_buff::make_camera_transform(camera, stamp);
    require(transform.at("parent_frame_id") == "world" &&
            transform.at("child_frame_id") == "camera", "World must be a transform tree root");
    require((xyz(transform.at("translation")) - camera.t_world_camera).norm() < 1e-9,
            "Camera position is expressed in world");
    const auto & rotation = transform.at("rotation");
    const Eigen::Quaterniond camera_q(rotation.at("w"), rotation.at("x"),
                                     rotation.at("y"), rotation.at("z"));
    require((camera_q.toRotationMatrix() - camera.R_world_camera).norm() < 1e-9,
            "Camera to world rotation direction");
    require(transform.at("timestamp").at("nsec") == 123456789, "Transform timestamp precision");
    const auto scene = auto_buff::make_rune_scene(observed, target, {}, stamp, 0.2);
    observed.camera_model_pose = auto_buff::InactiveTargets::CameraModelPose{
      Vec(0.1, 0.2, 0.3), Vec(1.0, 2.0, 3.0)};
    const auto telemetry = auto_buff::make_rune_telemetry(observed, target, 0.2);
    require(telemetry.at("rvec_x") == 0.1 && telemetry.at("rvec_y") == 0.2 &&
            telemetry.at("rvec_z") == 0.3, "Rotation vector reaches telemetry");
    require(telemetry.at("tvec_x") == 1.0 && telemetry.at("tvec_y") == 2.0 &&
            telemetry.at("tvec_z") == 3.0, "Translation vector reaches telemetry");
    require(auto_buff::make_rune_telemetry(observed, std::nullopt, 0.2).contains("rvec_x"),
            "Pose telemetry must not depend on motion tracking");
    require(!auto_buff::make_rune_telemetry(std::nullopt, target, 0.2).contains("rvec_x"),
            "Missing observation must not publish stale pose");
    require(telemetry.at("observation_valid") == 1, "Telemetry observation status");
    require(telemetry.at("tracking_valid") == 1, "Telemetry tracking status");
    require(std::abs(telemetry.at("phase_rad").get<double>() - target.phase) < 1e-9,
            "Telemetry phase");
    require(
      std::abs(telemetry.at("predicted_phase_rad").get<double>() - 0.2) < 1e-9,
      "Telemetry predicted phase");
    require(nlohmann::json::parse(tools::kBuffTelemetrySchema).at("title") ==
              "auto_buff.Telemetry",
            "Telemetry schema");
    const auto & plane = entity(scene, "rune_plane");
    require(plane.at("timestamp").at("sec") == 1700000000, "Seconds precision");
    require(plane.at("timestamp").at("nsec") == 123456789, "Nanoseconds precision");
    const auto & orientation = plane.at("cubes")[0].at("pose").at("orientation");
    const Eigen::Quaterniond q(orientation.at("w"), orientation.at("x"),
                               orientation.at("y"), orientation.at("z"));
    require((q * Vec::UnitZ() - normal).norm() < 1e-9, "Plane orientation");
    for (const auto & p : plane.at("lines")[0].at("points")) {
      const Vec offset = xyz(p) - center;
      require(std::abs(offset.dot(normal)) < 1e-9, "Orbit must lie in fitted plane");
      require(std::abs(offset.norm() - 0.7) < 1e-9, "Orbit radius");
    }
    const auto & blades = entity(scene, "five_blade_reference").at("cubes");
    require(blades.size() == 5, "Five rune blades are cuboids");
    const Vec blade_size = xyz(blades.front().at("size"));
    require(std::abs(blade_size.x() - (0.85 - 0.17394)) < 1e-9, "Blade radial length");
    require(std::abs(blade_size.y() - 0.30) < 1e-9, "Blade lateral width");
    const Vec first_blade_center = xyz(blades.front().at("pose").at("position"));
    require(
      (first_blade_center - (center + 0.5 * (0.17394 + 0.85) * radial)).norm() < 1e-9,
      "Blade rotates around R center");
    const Vec r_marker = xyz(plane.at("spheres").front().at("pose").at("position"));
    require((r_marker - center).norm() < 1e-9, "R marker remains a point");
    auto rotated_target = target;
    rotated_target.phase = 0.35;
    const auto rotated_scene =
      auto_buff::make_rune_scene(observed, rotated_target, {}, stamp, 0.2);
    const Vec rotated_blade_center = xyz(
      entity(rotated_scene, "five_blade_reference").at("cubes").front().at("pose").at("position"));
    const Vec rotated_radial =
      std::cos(rotated_target.phase) * radial +
      std::sin(rotated_target.phase) * normal.cross(radial);
    require(
      (rotated_blade_center -
       (center + 0.5 * (0.17394 + 0.85) * rotated_radial)).norm() < 1e-9,
      "Blade cuboids follow tracked phase around R center");
    const Vec predicted = xyz(entity(scene, "motion_prediction").at("spheres")[0]
                               .at("pose").at("position"));
    const Vec expected = center + 0.7 * (std::cos(0.2) * radial + std::sin(0.2) * normal.cross(radial));
    require((predicted - expected).norm() < 1e-9, "Prediction direction and duration");
    const auto lost = auto_buff::make_rune_scene({}, {}, {}, stamp + 1);
    require(lost.at("entities").size() == 1, "Lost scene contains only world/camera axes");
    require(lost.at("deletions")[0].at("type") == 1, "Lost target must clear old entities");
    const auto before_tracking = auto_buff::make_rune_scene(observed, {}, {}, stamp);
    require(entity(before_tracking, "rune_plane").at("cubes").size() == 1,
            "Plane should be visible before tracker initializes");
    observed.power_rune_plane.normal().setZero();
    const auto invalid = auto_buff::make_rune_scene(observed, {}, {}, stamp);
    require(invalid.dump().find("null") == std::string::npos, "Degenerate normal produces no NaN");
    if (argc == 3 && std::string(argv[1]) == "--dump") {
      std::ofstream(argv[2]) << Json::array({scene, lost, before_tracking, invalid}).dump(2);
    }
    if (argc == 3 && std::string(argv[1]) == "--serve") {
      tools::FoxgloveServer server("127.0.0.1", std::stoi(argv[2]), {
        {1, "/buff/scene", "foxglove.SceneUpdate", tools::kFoxgloveSceneSchema},
        {2, "/buff/transforms", "foxglove.FrameTransform", tools::kFoxgloveTransformSchema},
        {3, "/buff/telemetry", "auto_buff.Telemetry", tools::kBuffTelemetrySchema}});
      for (int i = 0; i < 200; ++i) {
        server.publish(stamp, {{2, transform.dump()}, {1, scene.dump()}, {3, telemetry.dump()}});
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
      }
    }
    std::cout << "Rune scene geometry and lifecycle checks passed\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
