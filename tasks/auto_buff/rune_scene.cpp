#include "rune_scene.hpp"

#include <cmath>
#include <string>

namespace auto_buff
{
namespace
{
using Json = nlohmann::json;
using Vec = Eigen::Vector3d;
constexpr double kPi = 3.14159265358979323846;
// Derived from PowerRunePlane::kPlanePoints: blade samples span radius
// 0.17394..0.85 m and lateral coordinates -0.15..0.15 m.
constexpr double kBladeInnerRadius = 0.17394;
constexpr double kBladeOuterRadius = 0.85;
constexpr double kBladeWidth = 0.30;
constexpr double kBladeThickness = 0.08;

Json xyz(const Vec & p) { return {{"x", p.x()}, {"y", p.y()}, {"z", p.z()}}; }
Json color(double r, double g, double b, double a = 1.0)
{ return {{"r", r}, {"g", g}, {"b", b}, {"a", a}}; }
Json pose(const Vec & p, const Eigen::Quaterniond & q = Eigen::Quaterniond::Identity())
{
  return {{"position", xyz(p)},
          {"orientation", {{"x", q.x()}, {"y", q.y()}, {"z", q.z()}, {"w", q.w()}}}};
}
Json entity(const std::string & id, const Json & stamp)
{
  Json e = {{"id", id}, {"frame_id", "world"}, {"timestamp", stamp},
            {"lifetime", {{"sec", 0}, {"nsec", 0}}}, {"frame_locked", false}};
  for (const char * field : {"metadata", "arrows", "cubes", "spheres", "cylinders",
                            "lines", "triangles", "texts", "models"})
    e[field] = Json::array();
  return e;
}
void sphere(Json & e, const Vec & p, double diameter, const Json & c)
{
  e["spheres"].push_back({{"pose", pose(p)}, {"size", xyz(Vec::Constant(diameter))}, {"color", c}});
}
void line(Json & e, const Json & points, const Json & c, int type = 0)
{
  e["lines"].push_back({{"type", type}, {"pose", pose(Vec::Zero())}, {"thickness", 2.0},
    {"scale_invariant", true}, {"points", points}, {"color", c},
    {"colors", Json::array()}, {"indices", Json::array()}});
}
void arrow(Json & e, const Vec & p, const Vec & direction, double length, const Json & c)
{
  const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Vec::UnitX(), direction);
  e["arrows"].push_back({{"pose", pose(p, q)}, {"shaft_length", length * 0.8},
    {"shaft_diameter", 0.012}, {"head_length", length * 0.2}, {"head_diameter", 0.04},
    {"color", c}});
}
void label(Json & e, const Vec & p, const std::string & text, const Json & c)
{
  e["texts"].push_back({{"pose", pose(p)}, {"billboard", true}, {"font_size", 14.0},
    {"scale_invariant", true}, {"color", c}, {"text", text}});
}
bool valid(const Vec & p) { return p.allFinite(); }
void add_vector(Json & data, const std::string & prefix, const Vec & value)
{
  if (!valid(value)) return;
  data[prefix + "_x_m"] = value.x();
  data[prefix + "_y_m"] = value.y();
  data[prefix + "_z_m"] = value.z();
}
}  // namespace

Json make_camera_transform(const CameraPose & camera, std::uint64_t timestamp_ns)
{
  const auto camera_pose = pose(camera.t_world_camera, Eigen::Quaterniond(camera.R_world_camera));
  return {{"timestamp", {{"sec", timestamp_ns / 1000000000ULL},
                          {"nsec", timestamp_ns % 1000000000ULL}}},
          {"parent_frame_id", "world"}, {"child_frame_id", "camera"},
          {"translation", camera_pose.at("position")},
          {"rotation", camera_pose.at("orientation")}};
}

Json make_rune_scene(
  const std::optional<InactiveTargets> & observed, const std::optional<RuneTarget> & tracked,
  const CameraPose & camera, std::uint64_t timestamp_ns, double prediction_seconds)
{
  const Json stamp = {{"sec", timestamp_ns / 1000000000ULL},
                      {"nsec", timestamp_ns % 1000000000ULL}};
  // Each message is a complete snapshot, including explicit removal of vanished entities.
  Json scene = {{"deletions", Json::array({{{"timestamp", stamp}, {"type", 1}, {"id", ""}}})},
                {"entities", Json::array()}};
  const auto red = color(1, 0.15, 0.1), green = color(0.1, 1, 0.25);
  const auto blue = color(0.15, 0.5, 1), yellow = color(1, 0.85, 0.1);
  auto axes = entity("world_camera", stamp);
  arrow(axes, Vec::Zero(), Vec::UnitX(), 0.5, red);
  arrow(axes, Vec::Zero(), Vec::UnitY(), 0.5, green);
  arrow(axes, Vec::Zero(), Vec::UnitZ(), 0.5, blue);
  label(axes, Vec(0, 0, 0.6), "world (m): X forward / Y left / Z up", color(1, 1, 1));
  if (valid(camera.t_world_camera) && camera.R_world_camera.allFinite()) {
    sphere(axes, camera.t_world_camera, 0.06, color(1, 1, 1));
    for (int i = 0; i < 3; ++i)
      arrow(axes, camera.t_world_camera, camera.R_world_camera.col(i), 0.25,
            i == 0 ? red : (i == 1 ? green : blue));
    label(axes, camera.t_world_camera + Vec(0, 0, 0.3), "camera: Z optical axis", color(1, 1, 1));
  }
  scene["entities"].push_back(std::move(axes));

  // Draw the fitted observation even before the motion tracker initializes.
  if (observed && !observed->rune_pieces.empty()) {
    const Vec normal = observed->power_rune_plane.normal();
    const Vec center = observed->rune_pieces.front().rune_center;
    const Vec radial = observed->rune_pieces.front().armor_center - center;
    if (valid(center) && valid(normal) && normal.norm() > 1e-6 &&
        valid(radial) && radial.norm() > 1e-6) {
      const Vec n = normal.normalized();
      const Vec projected_radial = radial - radial.dot(n) * n;
      const Vec u = projected_radial.norm() > 1e-6 ? Vec(projected_radial.normalized()) : n.unitOrthogonal();
      const Vec v = n.cross(u).normalized();
      const double radius = radial.norm();
      Eigen::Matrix3d rotation;
      rotation.col(0) = u; rotation.col(1) = v; rotation.col(2) = n;
      auto plane = entity("rune_plane", stamp);
      plane["cubes"].push_back({{"pose", pose(center, Eigen::Quaterniond(rotation))},
        {"size", xyz(Vec(2 * (radius + 0.25), 2 * (radius + 0.25), 0.006))},
        {"color", color(0.1, 0.65, 1, 0.16)}});
      arrow(plane, center, n, 0.5, blue);
      sphere(plane, center, 0.09, blue);
      label(plane, center + 0.15 * n, "R center / fitted plane normal", blue);
      Json circle = Json::array();
      for (int i = 0; i < 96; ++i) {
        const double a = 2 * kPi * i / 96;
        circle.push_back(xyz(center + radius * (std::cos(a) * u + std::sin(a) * v)));
      }
      line(plane, circle, blue, 1);  // LINE_LOOP
      plane["metadata"].push_back({{"key", "normal_world"}, {"value", xyz(n).dump()}});
      plane["metadata"].push_back({{"key", "plane_offset_m"},
                                   {"value", std::to_string(observed->power_rune_plane.offset())}});
      scene["entities"].push_back(std::move(plane));

      Vec model_center = center;
      Vec model_normal = n;
      Vec model_radial = u;
      if (
        tracked && valid(tracked->rune_center) && valid(tracked->start_vector) &&
        valid(tracked->rune_plane_world_normal) &&
        tracked->start_vector.norm() > 1e-6 &&
        tracked->rune_plane_world_normal.norm() > 1e-6 && std::isfinite(tracked->phase)) {
        model_center = tracked->rune_center;
        model_normal = tracked->rune_plane_world_normal.normalized();
        Vec phase_zero = tracked->start_vector;
        phase_zero -= phase_zero.dot(model_normal) * model_normal;
        if (phase_zero.norm() > 1e-6) {
          phase_zero.normalize();
          model_radial = std::cos(tracked->phase) * phase_zero +
                         std::sin(tracked->phase) * model_normal.cross(phase_zero);
          model_radial.normalize();
        }
      }

      const Vec model_tangent = model_normal.cross(model_radial).normalized();
      const double blade_center_radius = 0.5 * (kBladeInnerRadius + kBladeOuterRadius);
      const double blade_length = kBladeOuterRadius - kBladeInnerRadius;
      auto model = entity("five_blade_reference", stamp);
      for (int i = 0; i < 5; ++i) {
        const double a = 2 * kPi * i / 5;
        const Vec radial = std::cos(a) * model_radial + std::sin(a) * model_tangent;
        const Vec tangent = model_normal.cross(radial).normalized();
        Eigen::Matrix3d blade_rotation;
        blade_rotation.col(0) = radial;
        blade_rotation.col(1) = tangent;
        blade_rotation.col(2) = model_normal;
        model["cubes"].push_back({
          {"pose", pose(
                     model_center + blade_center_radius * radial,
                     Eigen::Quaterniond(blade_rotation))},
          {"size", xyz(Vec(blade_length, kBladeWidth, kBladeThickness))},
          {"color", color(0.65, 0.65, 0.65, 0.6)}});
      }
      scene["entities"].push_back(std::move(model));
    }
    auto measured = entity("observed_inactive_blades", stamp);
    for (std::size_t i = 0; i < observed->rune_pieces.size(); ++i) {
      const Vec p = observed->rune_pieces[i].armor_center;
      if (!valid(p)) continue;
      sphere(measured, p, 0.085, red);
      label(measured, p + Vec(0, 0, 0.12), "observed inactive " + std::to_string(i), red);
    }
    scene["entities"].push_back(std::move(measured));
  }

  if (tracked && valid(tracked->rune_center) && valid(tracked->armor_module_center) &&
      valid(tracked->start_vector) && valid(tracked->rune_plane_world_normal)) {
    auto target = entity("tracked_target", stamp);
    sphere(target, tracked->armor_module_center, 0.055, yellow);
    label(target, tracked->armor_module_center + Vec(0, 0, 0.2), "tracked armor", yellow);
    if (tracked->start_vector.norm() > 1e-6)
      arrow(target, tracked->rune_center, tracked->start_vector, 0.4, yellow);
    target["metadata"] = Json::array({
      {{"key", "phase_rad"}, {"value", std::to_string(tracked->phase)}},
      {{"key", "angular_velocity_rad_s"}, {"value", std::to_string(tracked->angular_velocity)}},
      {{"key", "prediction_seconds"}, {"value", std::to_string(prediction_seconds)}},
      {{"key", "armor_center_world_m"}, {"value", xyz(tracked->armor_module_center).dump()}}});
    scene["entities"].push_back(std::move(target));
    if (std::isfinite(prediction_seconds) && prediction_seconds > 0) {
      auto future = entity("motion_prediction", stamp);
      Json points = Json::array();
      for (int i = 0; i <= 32; ++i) {
        const auto dt = std::chrono::duration_cast<RuneTimestamp::duration>(
          std::chrono::duration<double>(prediction_seconds * i / 32));
        const Vec p = predict_armor_position(*tracked, tracked->capture_timestamp + dt);
        if (!valid(p)) { points.clear(); break; }
        points.push_back(xyz(p));
      }
      if (!points.empty()) {
        line(future, points, green);
        const auto & last = points.back();
        const Vec p(last["x"].get<double>(), last["y"].get<double>(), last["z"].get<double>());
        sphere(future, p, 0.065, green);
        label(future, p + Vec(0, 0, -0.15), "motion preview +" +
              std::to_string(prediction_seconds) + " s", green);
        scene["entities"].push_back(std::move(future));
      }
    }
  }
  return scene;
}

Json make_rune_telemetry(
  const std::optional<InactiveTargets> & observed, const std::optional<RuneTarget> & tracked,
  double prediction_seconds)
{
  Json data = {
    {"observation_valid", observed.has_value() ? 1 : 0},
    {"tracking_valid", tracked.has_value() ? 1 : 0},
    {"mode_big", tracked ? (tracked->is_big_rune ? 1 : 0)
                          : (observed && observed->is_big_rune ? 1 : 0)},
    {"inactive_target_count",
     tracked ? tracked->inactive_target_num
             : (observed ? static_cast<int>(observed->rune_pieces.size()) : 0)},
    {"prediction_seconds", prediction_seconds}};

  if (observed) {
    if (observed->camera_model_pose) {
      const auto & pose = *observed->camera_model_pose;
      if (valid(pose.rvec) && valid(pose.tvec)) {
        data["rvec_x"] = pose.rvec.x();
        data["rvec_y"] = pose.rvec.y();
        data["rvec_z"] = pose.rvec.z();
        data["tvec_x"] = pose.tvec.x();
        data["tvec_y"] = pose.tvec.y();
        data["tvec_z"] = pose.tvec.z();
      }
    }
    const Vec normal = observed->power_rune_plane.normal();
    if (valid(normal)) {
      data["plane_normal_x"] = normal.x();
      data["plane_normal_y"] = normal.y();
      data["plane_normal_z"] = normal.z();
    }
    const double offset = observed->power_rune_plane.offset();
    if (std::isfinite(offset)) data["plane_offset_m"] = offset;
    if (!observed->rune_pieces.empty()) {
      add_vector(data, "observed_rune_center", observed->rune_pieces.front().rune_center);
      add_vector(data, "observed_armor_center", observed->rune_pieces.front().armor_center);
    }
  }

  if (!tracked) return data;

  add_vector(data, "tracked_rune_center", tracked->rune_center);
  add_vector(data, "tracked_armor_center", tracked->armor_module_center);
  if (std::isfinite(tracked->phase)) {
    data["phase_rad"] = tracked->phase;
    data["phase_deg"] = tracked->phase * 180.0 / kPi;
  }
  if (std::isfinite(tracked->angular_velocity))
    data["angular_velocity_rad_s"] = tracked->angular_velocity;

  if (std::isfinite(prediction_seconds) && prediction_seconds >= 0.0) {
    const auto duration = std::chrono::duration_cast<RuneTimestamp::duration>(
      std::chrono::duration<double>(prediction_seconds));
    const RuneTimestamp prediction_time = tracked->capture_timestamp + duration;
    const double predicted_phase = predict_phase(*tracked, prediction_time);
    if (std::isfinite(predicted_phase)) data["predicted_phase_rad"] = predicted_phase;
    add_vector(data, "predicted_armor_center", predict_armor_position(*tracked, prediction_time));
  }

  if (tracked->is_big_rune) {
    const BigRuneMotionModel & model = tracked->big_rune_motion_model;
    data["model_phase_cos_coefficient"] = model.phase_cos_coefficient;
    data["model_phase_sin_coefficient"] = model.phase_sin_coefficient;
    data["model_phase_linear_velocity_rad_s"] = model.phase_linear_velocity;
    data["model_phase_constant_offset_rad"] = model.phase_constant_offset_radians;
    data["model_speed_angular_frequency_rad_s"] = model.speed_angular_frequency;
    data["model_speed_amplitude_rad_s"] = model.speed_amplitude;
    data["model_speed_phase_shift_rad"] = model.speed_phase_shift;
  }
  return data;
}
}  // namespace auto_buff
