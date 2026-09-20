#include "phase_motion_estimator.hpp"

#include <cmath>
#include <utility>

namespace auto_buff
{
namespace
{
constexpr double kGeometryEpsilon = 1e-9;

bool finite_vector(const Eigen::Vector3d & vector)
{
  return vector.array().isFinite().all();
}
}  // namespace

PhaseMotionEstimator::PhaseMotionEstimator(const std::string & config_path)
: small_estimator_(config_path), big_estimator_(config_path)
{
}

void PhaseMotionEstimator::reset()
{
  small_estimator_.reset();
  big_estimator_.reset();
  last_mode_was_big_.reset();
}

std::vector<RuneCandidateTarget> PhaseMotionEstimator::generate_candidate_targets(
  const InactiveTargets & inactive_targets)
{
  std::vector<RuneCandidateTarget> candidates;
  Eigen::Vector3d normal = inactive_targets.power_rune_plane.normal();
  if (!finite_vector(normal) || normal.norm() <= kGeometryEpsilon) return candidates;
  normal.normalize();

  // sp_vision world is z-up. The source project uses (0, -1, 0) because its car frame is y-down.
  Eigen::Vector3d start_vector = normal.cross(Eigen::Vector3d::UnitZ()); // start_vector同时垂直于法向量和z轴（能量机关平面与世界系xy面相交线）
  if (start_vector.norm() <= kGeometryEpsilon)
    start_vector = normal.cross(Eigen::Vector3d::UnitY());
  if (start_vector.norm() <= kGeometryEpsilon) return candidates;
  start_vector.normalize(); // 定义零相位参考方向

  for (const auto & piece : inactive_targets.rune_pieces) {
    Eigen::Vector3d radial = piece.armor_center - piece.rune_center; // 靶心到R标向量方向
    radial -= normal * radial.dot(normal);
    if (!finite_vector(radial) || radial.norm() <= kGeometryEpsilon) continue;
    radial.normalize();

    RuneCandidateTarget candidate;
    candidate.rune_center = piece.rune_center;
    candidate.armor_module_center = piece.armor_center;
    candidate.start_vector = start_vector;
    candidate.rune_plane_world_normal = normal;
    candidate.phase = // 从零相位到靶心->R标向量夹角
      std::atan2(normal.dot(start_vector.cross(radial)), start_vector.dot(radial));
    candidate.capture_timestamp = inactive_targets.capture_timestamp;
    candidates.emplace_back(std::move(candidate));
  }
  return candidates;
}

std::optional<RuneTarget> PhaseMotionEstimator::estimate_phase_motion(
  const InactiveTargets & inactive_targets)
{
  const std::size_t max_targets = inactive_targets.is_big_rune ? 2U : 1U;
  if (inactive_targets.rune_pieces.empty() ||
      inactive_targets.rune_pieces.size() > max_targets) {
    return std::nullopt;
  }

  if (
    last_mode_was_big_ &&
    *last_mode_was_big_ != inactive_targets.is_big_rune) { // 如果大符小符模式发生切换清空所有历史状态
    small_estimator_.reset();
    big_estimator_.reset();
  }
  last_mode_was_big_ = inactive_targets.is_big_rune;

  std::vector<RuneCandidateTarget> candidates = generate_candidate_targets(inactive_targets);
  if (candidates.empty()) return std::nullopt;

  if (inactive_targets.is_big_rune) {
    return big_estimator_.estimate(
      candidates, static_cast<int>(inactive_targets.rune_pieces.size()));
  }
  return small_estimator_.estimate(candidates.front());
}
}  // namespace auto_buff
