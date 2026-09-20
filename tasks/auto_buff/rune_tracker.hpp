#ifndef AUTO_BUFF__RUNE_TRACKER_HPP
#define AUTO_BUFF__RUNE_TRACKER_HPP

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rune_camera.hpp"
#include "tools/math_tools.hpp"

namespace cv
{
class Mat;
}

namespace auto_buff
{
struct RuneDecisionTarget;
using RuneTimestamp = std::chrono::steady_clock::time_point;

struct InactiveTargets
{
  struct CameraModelPose
  {
    Eigen::Vector3d rvec;  // Rodrigues rotation vector, radians (model to camera).
    Eigen::Vector3d tvec;  // Model origin in camera coordinates, meters.
  };
  struct RunePiece
  {
    Eigen::Vector3d rune_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d armor_center = Eigen::Vector3d::Zero();
  };

  std::vector<RunePiece> rune_pieces;
  Eigen::Hyperplane<double, 3> power_rune_plane;
  RuneTimestamp capture_timestamp{};
  bool is_big_rune = false;
  std::optional<CameraModelPose> camera_model_pose;
};

struct RuneCandidateTarget
{
  Eigen::Vector3d rune_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d armor_module_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d start_vector = Eigen::Vector3d::UnitY();
  Eigen::Vector3d rune_plane_world_normal = Eigen::Vector3d::UnitX();
  double phase = 0.0;
  RuneTimestamp capture_timestamp{};
};

inline double normalize_rune_phase(double phase)
{
  return tools::limit_rad(phase);
}

inline double rune_phase_difference(double current, double previous)
{
  return tools::limit_rad(current - previous);
}

struct BigRuneMotionModel
{
  RuneTimestamp reference_timestamp{};
  double phase_cos_coefficient = 0.0;
  double phase_sin_coefficient = 0.0;
  double phase_linear_velocity = 0.0;
  double phase_constant_offset_radians = 0.0;
  double speed_angular_frequency = 0.0;
  double speed_amplitude = 0.0;
  double speed_phase_shift = 0.0;
};

struct RuneTarget
{
  Eigen::Vector3d rune_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d armor_module_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d start_vector = Eigen::Vector3d::UnitY();
  Eigen::Vector3d rune_plane_world_normal = Eigen::Vector3d::UnitX();
  double phase = 0.0;
  double angular_velocity = 0.0;
  BigRuneMotionModel big_rune_motion_model;
  RuneTimestamp capture_timestamp{};
  int inactive_target_num = 0;
  bool is_big_rune = false;
};

class RuneTargetTracker
{
public:
  explicit RuneTargetTracker(const std::string & config_path);
  ~RuneTargetTracker();

  RuneTargetTracker(const RuneTargetTracker &) = delete;
  RuneTargetTracker & operator=(const RuneTargetTracker &) = delete;
  RuneTargetTracker(RuneTargetTracker &&) noexcept;
  RuneTargetTracker & operator=(RuneTargetTracker &&) noexcept;

  std::optional<RuneTarget> track(
    const std::optional<InactiveTargets> & inactive_targets, cv::Mat & visualization_image,
    const CameraPose & camera_pose, const RuneCamera & camera, bool to_now = true);

  // Latest decision from track(); lifetime is bounded by this tracker.
  const RuneDecisionTarget & decision() const;
  // Feed the ballistic result back using the same decision timestamp as track().
  bool allow_fire(bool ballistic_solution_valid);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

double predict_phase(const RuneTarget & target, RuneTimestamp timestamp);
Eigen::Vector3d predict_armor_position(const RuneTarget & target, RuneTimestamp timestamp);
}  // namespace auto_buff

#endif  // AUTO_BUFF__RUNE_TRACKER_HPP
