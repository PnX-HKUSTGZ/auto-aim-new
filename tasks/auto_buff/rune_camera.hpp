#ifndef AUTO_BUFF__RUNE_CAMERA_HPP
#define AUTO_BUFF__RUNE_CAMERA_HPP

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include <optional>
#include <string>

namespace auto_buff
{
struct CameraPose
{
  Eigen::Matrix3d R_world_camera = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_world_camera = Eigen::Vector3d::Zero();  // metres
};

// Maintains the camera pose used by the rune pipeline from the sampled gimbal orientation.
class RuneCamera
{
public:
  explicit RuneCamera(const std::string & config_path);

  void set_gimbal_orientation(const Eigen::Quaterniond & orientation);

  const Eigen::Matrix3d & R_gimbal2world() const;
  CameraPose pose() const;
  const cv::Mat & camera_matrix() const;
  const cv::Mat & distort_coeffs() const;
  std::optional<cv::Point2f> project_world_point(
    const Eigen::Vector3d & point_in_world, const CameraPose & camera_pose) const;

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_camera2gimbal_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_camera2gimbal_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_gimbal2world_ = Eigen::Matrix3d::Identity();
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__RUNE_CAMERA_HPP
