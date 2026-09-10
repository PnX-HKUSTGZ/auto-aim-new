#include "rune_camera.hpp"

#include <yaml-cpp/yaml.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace auto_buff
{
RuneCamera::RuneCamera(const std::string & config_path)
{
  const YAML::Node yaml = YAML::LoadFile(config_path);
  const auto R_gimbal2imubody_data =
    yaml["R_gimbal2imubody"].as<std::vector<double>>();
  const auto R_camera2gimbal_data =
    yaml["R_camera2gimbal"].as<std::vector<double>>();
  const auto t_camera2gimbal_data =
    yaml["t_camera2gimbal"].as<std::vector<double>>();
  const auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  const auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();

  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(
    R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(
    R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Map<const Eigen::Vector3d>(t_camera2gimbal_data.data());

  const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(
    camera_matrix_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  distort_coeffs_ = cv::Mat(1, static_cast<int>(distort_coeffs_data.size()), CV_64F);
  std::copy(
    distort_coeffs_data.begin(), distort_coeffs_data.end(), distort_coeffs_.ptr<double>());
}

void RuneCamera::set_gimbal_orientation(const Eigen::Quaterniond & orientation)
{
  const Eigen::Matrix3d R_imubody2imuabs = orientation.toRotationMatrix();
  R_gimbal2world_ =
    R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

const Eigen::Matrix3d & RuneCamera::R_gimbal2world() const
{
  return R_gimbal2world_;
}

CameraPose RuneCamera::pose() const
{
  CameraPose camera_pose;
  camera_pose.R_world_camera = R_gimbal2world_ * R_camera2gimbal_;
  camera_pose.t_world_camera = R_gimbal2world_ * t_camera2gimbal_;
  return camera_pose;
}

const cv::Mat & RuneCamera::camera_matrix() const
{
  return camera_matrix_;
}

const cv::Mat & RuneCamera::distort_coeffs() const
{
  return distort_coeffs_;
}

std::optional<cv::Point2f> RuneCamera::project_world_point(
  const Eigen::Vector3d & point_in_world, const CameraPose & camera_pose) const
{
  const Eigen::Vector3d point_in_camera =
    camera_pose.R_world_camera.transpose() * (point_in_world - camera_pose.t_world_camera);
  if (!point_in_camera.array().isFinite().all() || point_in_camera.z() <= 1e-6)
    return std::nullopt;

  const std::vector<cv::Point3d> object_points{{
    point_in_camera.x(), point_in_camera.y(), point_in_camera.z()}};
  std::vector<cv::Point2d> image_points;
  cv::projectPoints(
    object_points, cv::Vec3d::all(0.0), cv::Vec3d::all(0.0), camera_matrix_,
    distort_coeffs_, image_points);
  if (image_points.empty() || !std::isfinite(image_points.front().x) ||
      !std::isfinite(image_points.front().y)) {
    return std::nullopt;
  }
  return cv::Point2f(
    static_cast<float>(image_points.front().x),
    static_cast<float>(image_points.front().y));
}
}  // namespace auto_buff
