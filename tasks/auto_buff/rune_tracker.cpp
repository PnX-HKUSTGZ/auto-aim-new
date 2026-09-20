#include "rune_tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <utility>

#include "phase_motion_estimator.hpp"
#include "rune_decision_state_machine.hpp"

namespace auto_buff
{
namespace
{
constexpr auto kMotionPredictionDuration = std::chrono::milliseconds(100);
}  // namespace

double predict_phase(const RuneTarget & target, RuneTimestamp timestamp)
{
  const double dt = tools::delta_time(timestamp, target.capture_timestamp);
  if (!target.is_big_rune) return target.phase + target.angular_velocity * dt;

  const BigRuneMotionModel & model = target.big_rune_motion_model;
  const double model_time = tools::delta_time(timestamp, model.reference_timestamp);
  return model.phase_cos_coefficient * std::cos(model.speed_angular_frequency * model_time) +
         model.phase_sin_coefficient * std::sin(model.speed_angular_frequency * model_time) +
         model.phase_linear_velocity * model_time + model.phase_constant_offset_radians;
}

Eigen::Vector3d predict_armor_position(const RuneTarget & target, RuneTimestamp timestamp)
{
  const double phase = predict_phase(target, timestamp);
  const Eigen::Vector3d radial =
    target.start_vector * std::cos(phase) +
    target.rune_plane_world_normal.cross(target.start_vector) * std::sin(phase);
  const double radius = (target.armor_module_center - target.rune_center).norm();
  return target.rune_center + radius * radial;
}

struct RuneTargetTracker::Impl
{
  explicit Impl(const std::string & config_path)
  : m_phase_motion_estimator(config_path), decision_state_machine_(config_path)
  {
    const YAML::Node yaml = YAML::LoadFile(config_path);
    const YAML::Node refine = yaml["buff_refine"];
    if (refine && refine["visualize_motion"])
      visualize_motion_ = refine["visualize_motion"].as<bool>();
    else if (refine && refine["visualize_plane"])
      visualize_motion_ = refine["visualize_plane"].as<bool>();
  }

  std::optional<RuneTarget> track(
    const std::optional<InactiveTargets> & inactive_targets, cv::Mat & visualization_image,
    const CameraPose & camera_pose, const RuneCamera & camera, bool to_now)
  {
    auto target = inactive_targets
                    ? m_phase_motion_estimator.estimate_phase_motion(*inactive_targets)
                    : std::nullopt;
    if (target) visualize_motion_estimate(visualization_image, *target, camera_pose, camera);
    decision_time_ = !to_now && inactive_targets
                       ? inactive_targets->capture_timestamp
                       : std::chrono::steady_clock::now();
    decision_ = decision_state_machine_.update(target, decision_time_, to_now); // 更新决策状态机
    return decision_.target;
  }

  void visualize_motion_estimate(
    cv::Mat & image, const RuneTarget & target, const CameraPose & camera_pose,
    const RuneCamera & camera) const
  {
    if (!visualize_motion_ || image.empty()) return;

    const auto observed = camera.project_world_point(target.armor_module_center, camera_pose);
    if (observed)
      cv::circle(image, *observed, 3, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);

    const auto predicted = camera.project_world_point(
      predict_armor_position(target, target.capture_timestamp + kMotionPredictionDuration),
      camera_pose);
    if (predicted)
      cv::circle(image, *predicted, 3, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
  }

  PhaseMotionEstimator m_phase_motion_estimator;
  RuneDecisionStateMachine decision_state_machine_;
  RuneDecisionTarget decision_;
  RuneTimestamp decision_time_{};
  bool visualize_motion_ = true;
};

RuneTargetTracker::RuneTargetTracker(const std::string & config_path)
: impl_(std::make_unique<Impl>(config_path))
{
}

RuneTargetTracker::~RuneTargetTracker() = default;
RuneTargetTracker::RuneTargetTracker(RuneTargetTracker &&) noexcept = default;
RuneTargetTracker & RuneTargetTracker::operator=(RuneTargetTracker &&) noexcept = default;

std::optional<RuneTarget> RuneTargetTracker::track(
  const std::optional<InactiveTargets> & inactive_targets, cv::Mat & visualization_image,
  const CameraPose & camera_pose, const RuneCamera & camera, bool to_now)
{
  return impl_->track(inactive_targets, visualization_image, camera_pose, camera, to_now);
}

const RuneDecisionTarget & RuneTargetTracker::decision() const
{
  return impl_->decision_;
}

bool RuneTargetTracker::allow_fire(bool ballistic_solution_valid)
{
  return impl_->decision_state_machine_.allow_fire(
    ballistic_solution_valid && impl_->decision_.state == RuneDecisionState::Tracking,
    impl_->decision_time_);
}
}  // namespace auto_buff
