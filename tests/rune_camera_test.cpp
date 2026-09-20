#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "tasks/auto_buff/rune_camera.hpp"

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main(int argc, char * argv[])
{
  try {
    const std::string config_path = argc > 1 ? argv[1] : "configs/standard3.yaml";
    const YAML::Node yaml = YAML::LoadFile(config_path);
    const auto gimbal_to_imu_data =
      yaml["R_gimbal2imubody"].as<std::vector<double>>();
    const auto camera_to_gimbal_data =
      yaml["R_camera2gimbal"].as<std::vector<double>>();
    const auto camera_translation_data =
      yaml["t_camera2gimbal"].as<std::vector<double>>();

    const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> gimbal_to_imu(
      gimbal_to_imu_data.data());
    const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_to_gimbal(
      camera_to_gimbal_data.data());
    const Eigen::Map<const Eigen::Vector3d> camera_translation(
      camera_translation_data.data());

    const Eigen::Quaterniond orientation(
      Eigen::AngleAxisd(0.37, Eigen::Vector3d(0.2, -0.4, 0.8).normalized()));
    auto_buff::RuneCamera camera(config_path);
    camera.set_gimbal_orientation(orientation);

    const Eigen::Matrix3d expected_gimbal_to_world =
      gimbal_to_imu.transpose() * orientation.toRotationMatrix() * gimbal_to_imu;
    const auto_buff::CameraPose pose = camera.pose();
    require(
      (camera.R_gimbal2world() - expected_gimbal_to_world).norm() < 1e-12,
      "Gimbal rotation must use the calibrated IMU reference frame");
    require(
      (pose.R_world_camera - expected_gimbal_to_world * camera_to_gimbal).norm() < 1e-12,
      "Camera rotation must use the calibrated IMU reference frame");
    require(
      (pose.t_world_camera - expected_gimbal_to_world * camera_translation).norm() < 1e-12,
      "Camera position must use the calibrated IMU reference frame");

    const Eigen::Vector3d optical_axis_point =
      pose.t_world_camera + pose.R_world_camera * Eigen::Vector3d(0.0, 0.0, 2.0);
    const auto optical_axis_pixel = camera.project_world_point(optical_axis_point, pose);
    require(optical_axis_pixel.has_value(), "Point in front of camera was not projected");
    require(
      std::abs(optical_axis_pixel->x - camera.camera_matrix().at<double>(0, 2)) < 1e-4 &&
      std::abs(optical_axis_pixel->y - camera.camera_matrix().at<double>(1, 2)) < 1e-4,
      "Optical axis did not project to the principal point");

    const Eigen::Vector3d point_behind_camera =
      pose.t_world_camera - pose.R_world_camera * Eigen::Vector3d(0.0, 0.0, 1.0);
    require(
      !camera.project_world_point(point_behind_camera, pose),
      "Point behind camera must not be projected");

    // 固定世界点在不同云台姿态下应恢复为同一坐标，不能随云台旋转。
    const Eigen::Vector3d fixed_world_point(2.0, -0.7, 0.4);
    for (double yaw : {0.0, 0.5, -0.8}) {
      const Eigen::Quaterniond initial_to_imu(
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()));
      camera.set_gimbal_orientation(initial_to_imu);
      // q 是“初始姿态到当前姿态”的旋转，当前 IMU 坐标 = R(q)^T * A * p_world。
      const Eigen::Vector3d point_in_imu =
        initial_to_imu.conjugate() * (gimbal_to_imu * fixed_world_point);
      const Eigen::Vector3d point_in_gimbal = gimbal_to_imu.transpose() * point_in_imu;
      require(
        (camera.R_gimbal2world() * point_in_gimbal - fixed_world_point).norm() < 1e-12,
        "A fixed world point must remain stationary as the gimbal rotates");
    }

    std::cout << "rune camera pose and projection checks passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
