#include "auto_aim_scene.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "tools/yaml.hpp"

namespace auto_aim
{
namespace
{
using Json = nlohmann::json;
using Vec = Eigen::Vector3d;

constexpr double kGravity = 9.8;
constexpr double kMinValue = 1e-6;
constexpr double kMaxExpArg = 50.0;
constexpr double kMaxFlightTime = 3.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kArmorPitch = 22.5 * kPi / 180.0;
constexpr int kTrajectorySamples = 80;

struct BallisticParameters
{
  double bullet_speed;
  double yaw;
  double pitch;  // Mathematical convention: muzzle-up is positive.
  double horizontal_distance;
  double air_resistance;
};

Json xyz(const Vec & point)
{
  return {{"x", point.x()}, {"y", point.y()}, {"z", point.z()}};
}

Json color(double red, double green, double blue, double alpha = 1.0)
{
  return {{"r", red}, {"g", green}, {"b", blue}, {"a", alpha}};
}

Json pose(
  const Vec & position, const Eigen::Quaterniond & orientation = Eigen::Quaterniond::Identity())
{
  return {
    {"position", xyz(position)},
    {"orientation",
     {{"x", orientation.x()},
      {"y", orientation.y()},
      {"z", orientation.z()},
      {"w", orientation.w()}}}};
}

Json entity(const std::string & id, const Json & timestamp)
{
  Json result = {
    {"id", id},
    {"frame_id", "world"},
    {"timestamp", timestamp},
    {"lifetime", {{"sec", 0}, {"nsec", 0}}},
    {"frame_locked", false}};
  for (const char * field : {
         "metadata", "arrows", "cubes", "spheres", "cylinders", "lines", "triangles",
         "texts", "models"})
    result[field] = Json::array();
  return result;
}

void sphere(Json & target, const Vec & position, double diameter, const Json & sphere_color)
{
  target["spheres"].push_back({
    {"pose", pose(position)},
    {"size", xyz(Vec::Constant(diameter))},
    {"color", sphere_color}});
}

void arrow(
  Json & target, const Vec & position, const Vec & direction, double length,
  const Json & arrow_color)
{
  if (!direction.allFinite() || direction.norm() <= kMinValue) return;
  const Eigen::Quaterniond orientation =
    Eigen::Quaterniond::FromTwoVectors(Vec::UnitX(), direction.normalized());
  target["arrows"].push_back({
    {"pose", pose(position, orientation)},
    {"shaft_length", length * 0.8},
    {"shaft_diameter", 0.012},
    {"head_length", length * 0.2},
    {"head_diameter", 0.035},
    {"color", arrow_color}});
}

void label(
  Json & target, const Vec & position, const std::string & text, const Json & text_color)
{
  target["texts"].push_back({
    {"pose", pose(position)},
    {"billboard", true},
    {"font_size", 14.0},
    {"scale_invariant", true},
    {"color", text_color},
    {"text", text}});
}

double flight_time(const BallisticParameters & ballistic)
{
  const double horizontal_speed = ballistic.bullet_speed * std::cos(ballistic.pitch);
  if (
    !std::isfinite(horizontal_speed) || horizontal_speed <= kMinValue ||
    !std::isfinite(ballistic.horizontal_distance) || ballistic.horizontal_distance <= 0.0) {
    return 0.0;
  }

  double result;
  if (std::isfinite(ballistic.air_resistance) && ballistic.air_resistance > kMinValue) {
    const double exponent = std::clamp(
      ballistic.air_resistance * ballistic.horizontal_distance, 0.0, kMaxExpArg);
    result =
      std::expm1(exponent) / (ballistic.air_resistance * horizontal_speed);
  } else {
    result = ballistic.horizontal_distance / horizontal_speed;
  }
  if (!std::isfinite(result) || result <= 0.0) return 0.0;
  return std::min(result, kMaxFlightTime);
}

Vec projectile_position(const BallisticParameters & ballistic, double time)
{
  const double horizontal_speed = ballistic.bullet_speed * std::cos(ballistic.pitch);
  const double vertical_speed = ballistic.bullet_speed * std::sin(ballistic.pitch);

  double horizontal_distance;
  double height;
  if (std::isfinite(ballistic.air_resistance) && ballistic.air_resistance > kMinValue) {
    const double resistance = ballistic.air_resistance;
    const double horizontal_log_argument = resistance * horizontal_speed * time + 1.0;
    horizontal_distance = std::log(horizontal_log_argument) / resistance;
    height =
      (vertical_speed + kGravity / resistance) * (-std::expm1(-resistance * time)) /
        resistance -
      kGravity * time / resistance;
  } else {
    horizontal_distance = horizontal_speed * time;
    height = vertical_speed * time - 0.5 * kGravity * time * time;
  }

  return {
    horizontal_distance * std::cos(ballistic.yaw),
    horizontal_distance * std::sin(ballistic.yaw), height};
}

double target_horizontal_distance(const std::list<Target> & targets, double yaw)
{
  if (targets.empty() || !std::isfinite(yaw)) return 0.0;

  double best_distance = 0.0;
  double best_yaw_error = std::numeric_limits<double>::infinity();
  for (const Eigen::Vector4d & armor : targets.front().armor_xyza_list()) {
    const Vec position = armor.head<3>();
    if (!position.allFinite()) continue;
    const double distance = std::hypot(position.x(), position.y());
    if (distance <= kMinValue) continue;
    const double yaw_error = std::abs(
      std::remainder(std::atan2(position.y(), position.x()) - yaw, 2.0 * kPi));
    if (yaw_error < best_yaw_error) {
      best_yaw_error = yaw_error;
      best_distance = distance;
    }
  }
  return best_distance;
}

void add_attitude(Json & scene, const Eigen::Quaterniond & attitude, const Json & timestamp)
{
  if (!attitude.coeffs().allFinite() || attitude.norm() <= kMinValue) return;
  const Eigen::Quaterniond normalized = attitude.normalized();
  auto gimbal = entity("gimbal_attitude", timestamp);
  arrow(gimbal, Vec::Zero(), normalized * Vec::UnitX(), 0.35, color(1.0, 0.1, 0.1));
  arrow(gimbal, Vec::Zero(), normalized * Vec::UnitY(), 0.35, color(0.1, 1.0, 0.2));
  arrow(gimbal, Vec::Zero(), normalized * Vec::UnitZ(), 0.35, color(0.1, 0.5, 1.0));
  label(gimbal, normalized * Vec(0.0, 0.0, 0.42), "gimbal attitude", color(1, 1, 1));
  gimbal["metadata"] = Json::array({
    {{"key", "quaternion_xyzw"},
     {"value",
      Json::array({normalized.x(), normalized.y(), normalized.z(), normalized.w()}).dump()}}});
  scene["entities"].push_back(std::move(gimbal));
}

void add_ballistic_trajectory(
  Json & scene, const BallisticParameters & ballistic, const Json & timestamp)
{
  if (
    !std::isfinite(ballistic.bullet_speed) || ballistic.bullet_speed <= kMinValue ||
    !std::isfinite(ballistic.yaw) || !std::isfinite(ballistic.pitch)) {
    return;
  }

  const double duration = flight_time(ballistic);
  if (duration <= 0.0) return;

  Json points = Json::array();
  for (int i = 0; i <= kTrajectorySamples; ++i) {
    const Vec point = projectile_position(
      ballistic, duration * static_cast<double>(i) / kTrajectorySamples);
    if (!point.allFinite()) return;
    points.push_back(xyz(point));
  }

  const Json trajectory_color = color(1.0, 0.25, 0.05, 0.95);
  auto trajectory = entity("ballistic_trajectory", timestamp);
  trajectory["lines"].push_back({
    {"type", 0},
    {"pose", pose(Vec::Zero())},
    {"thickness", 3.0},
    {"scale_invariant", true},
    {"points", std::move(points)},
    {"color", trajectory_color},
    {"colors", Json::array()},
    {"indices", Json::array()}});

  const Vec launch_direction(
    std::cos(ballistic.pitch) * std::cos(ballistic.yaw),
    std::cos(ballistic.pitch) * std::sin(ballistic.yaw),
    std::sin(ballistic.pitch));
  arrow(trajectory, Vec::Zero(), launch_direction, 0.4, trajectory_color);

  const Vec impact = projectile_position(ballistic, duration);
  sphere(trajectory, impact, 0.065, trajectory_color);
  label(trajectory, impact + Vec(0.0, 0.0, 0.12), "ballistic endpoint", trajectory_color);
  trajectory["metadata"] = Json::array({
    {{"key", "bullet_speed_m_s"}, {"value", std::to_string(ballistic.bullet_speed)}},
    {{"key", "yaw_rad"}, {"value", std::to_string(ballistic.yaw)}},
    {{"key", "pitch_rad_up_positive"}, {"value", std::to_string(ballistic.pitch)}},
    {{"key", "air_resistance"}, {"value", std::to_string(ballistic.air_resistance)}},
    {{"key", "flight_time_s"}, {"value", std::to_string(duration)}},
    {{"key", "horizontal_distance_m"},
     {"value", std::to_string(ballistic.horizontal_distance)}}});
  scene["entities"].push_back(std::move(trajectory));
}
}  // namespace

AutoAimVisualizer::AutoAimVisualizer(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);
  air_resistance_ = tools::read<double>(yaml, "air_resistance");
}

AutoAimVisualizer::AutoAimVisualizer(double air_resistance) : air_resistance_(air_resistance) {}

Json AutoAimVisualizer::make_gimbal_transform(
  const Eigen::Quaterniond & attitude, std::uint64_t timestamp_ns) const
{
  const Eigen::Quaterniond orientation =
    attitude.coeffs().allFinite() && attitude.norm() > kMinValue
      ? attitude.normalized()
      : Eigen::Quaterniond::Identity();
  return {
    {"timestamp",
     {{"sec", timestamp_ns / 1000000000ULL}, {"nsec", timestamp_ns % 1000000000ULL}}},
    {"parent_frame_id", "world"},
    {"child_frame_id", "gimbal"},
    {"translation", xyz(Vec::Zero())},
    {"rotation",
     {{"x", orientation.x()},
      {"y", orientation.y()},
      {"z", orientation.z()},
      {"w", orientation.w()}}}};
}

Json AutoAimVisualizer::make_scene(
  const std::list<Target> & targets, double bullet_speed, double yaw, double pitch,
  const Eigen::Quaterniond & attitude, std::uint64_t timestamp_ns) const
{
  const Json timestamp = {
    {"sec", timestamp_ns / 1000000000ULL}, {"nsec", timestamp_ns % 1000000000ULL}};
  Json scene = {
    {"deletions", Json::array({{{"timestamp", timestamp}, {"type", 1}, {"id", ""}}})},
    {"entities", Json::array()}};

  auto axes = entity("auto_aim_world", timestamp);
  arrow(axes, Vec::Zero(), Vec::UnitX(), 0.5, color(1.0, 0.1, 0.1));
  arrow(axes, Vec::Zero(), Vec::UnitY(), 0.5, color(0.1, 1.0, 0.2));
  arrow(axes, Vec::Zero(), Vec::UnitZ(), 0.5, color(0.1, 0.5, 1.0));
  label(axes, Vec(0.0, 0.0, 0.6), "world: X forward / Y left / Z up", color(1, 1, 1));
  scene["entities"].push_back(std::move(axes));
  add_attitude(scene, attitude, timestamp);

  std::size_t target_index = 0;
  for (const Target & tracked : targets) {
    const Eigen::VectorXd state = tracked.ekf_x();
    if (state.size() < 12) continue;
    const Vec center(state[0], state[2], state[4]);
    const Vec velocity(state[1], state[3], state[5]);
    if (!center.allFinite() || !velocity.allFinite()) continue;

    auto target = entity("auto_aim_target_" + std::to_string(target_index), timestamp);
    sphere(target, center, 0.09, color(0.15, 0.55, 1.0));
    const std::size_t name_index = static_cast<std::size_t>(tracked.name);
    const std::string target_name =
      name_index < ARMOR_NAMES.size() ? ARMOR_NAMES[name_index] : "unknown";
    label(target, center + Vec(0.0, 0.0, 0.18), target_name, color(1, 1, 1));
    if (velocity.norm() > kMinValue) {
      arrow(
        target, center, velocity,
        std::clamp(velocity.norm() * 0.25, 0.12, 0.7), color(0.2, 1.0, 0.3));
    }

    const auto armor_list = tracked.armor_xyza_list();
    Json orbit_points = Json::array();
    for (std::size_t armor_index = 0; armor_index < armor_list.size(); ++armor_index) {
      const Eigen::Vector4d & xyza = armor_list[armor_index];
      const Vec armor_center = xyza.head<3>();
      if (!armor_center.allFinite() || !std::isfinite(xyza[3])) continue;
      orbit_points.push_back(xyz(armor_center));

      const double yaw = xyza[3];
      const double pitch = kArmorPitch;
      const double sin_yaw = std::sin(yaw);
      const double cos_yaw = std::cos(yaw);
      const double sin_pitch = std::sin(pitch);
      const double cos_pitch = std::cos(pitch);
      Eigen::Matrix3d rotation;
      rotation << cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch,
        sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch,
        -sin_pitch, 0.0, cos_pitch;

      const double width =
        tracked.armor_type == ArmorType::big ? LARGE_ARMOR_WIDTH : SMALL_ARMOR_WIDTH;
      const double height =
        tracked.armor_type == ArmorType::big ? LARGE_ARMOR_HEIGHT : SMALL_ARMOR_HEIGHT;
      const bool matched = static_cast<int>(armor_index) == tracked.last_id;
      const Json armor_color =
        matched ? color(0.1, 1.0, 0.25, 0.9) : color(1.0, 0.75, 0.1, 0.7);
      target["cubes"].push_back({
        {"pose", pose(armor_center, Eigen::Quaterniond(rotation))},
        {"size", xyz(Vec(0.018, width, height))},
        {"color", armor_color}});
      arrow(target, armor_center, rotation.col(0), 0.10, armor_color);
    }
    if (orbit_points.size() > 1) {
      target["lines"].push_back({
        {"type", 1},
        {"pose", pose(Vec::Zero())},
        {"thickness", 1.5},
        {"scale_invariant", true},
        {"points", std::move(orbit_points)},
        {"color", color(0.4, 0.7, 1.0, 0.65)},
        {"colors", Json::array()},
        {"indices", Json::array()}});
    }
    target["metadata"] = Json::array({
      {{"key", "center_velocity_m_s"}, {"value", xyz(velocity).dump()}},
      {{"key", "angular_velocity_rad_s"}, {"value", std::to_string(state[7])}},
      {{"key", "radius_m"}, {"value", std::to_string(state[8])}},
      {{"key", "last_armor_id"}, {"value", std::to_string(tracked.last_id)}}});
    scene["entities"].push_back(std::move(target));
    ++target_index;
  }

  if (!std::isfinite(bullet_speed) || bullet_speed <= 0.0) {
    bullet_speed = 22.0;
  }
  const double horizontal_distance = target_horizontal_distance(targets, yaw);
  if (horizontal_distance > 0.0 && std::isfinite(pitch)) {
    // Planner/Gimbal command convention is muzzle-up negative.
    add_ballistic_trajectory(
      scene, {bullet_speed, yaw, -pitch, horizontal_distance, air_resistance_}, timestamp);
  }
  return scene;
}
}  // namespace auto_aim
