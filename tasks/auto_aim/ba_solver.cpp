// Created by Labor 2023.8.25
// Maintained by Chengfu Zou, Labor
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ba_solver.hpp"
// std
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
// g2o
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/slam3d/types_slam3d.h>
// 3rd party
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
// project
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim {
G2O_USE_OPTIMIZATION_LIBRARY(dense)

// 装甲板 yaw 优化的整体思路：
// 1. 上游 PnP 给出装甲板中心在相机系中的平移 t_camera_armor，以及原始旋转
//    R_armor2camera；上游还给出世界系 yaw 初值（通常先经过有界网格搜索）。
//    此处只优化一个标量 yaw，保持中心平移、四角的物体坐标和装甲板 pitch 不变。
// 2. 用装甲板型号确定宽高，在板坐标系构造四个角点；pitch 绕 Y 轴旋转，
//    普通装甲板取 +ARMOR_PITCH_RAD，前哨站取负值。yaw 绕世界系 Z 轴旋转。
//    对任一角点 P，旋转和平移后的相机系坐标为
//      P_c = R_world2camera * R_z(yaw) * R_y(pitch) * P + t_camera_armor，
//    其中 R_world2camera = R_camera2world.transpose()。
// 3. 将 P_c 透视除法得到归一化坐标，再施加前五个畸变系数
//    (k1, k2, p1, p2, k3) 和相机内参 K，得到预测像素 pi(P_c)。
//    每个角点的二维残差为“观测像素 - 预测像素”；四个固定角点各连一条
//    投影边到唯一可变的 yaw 顶点，信息矩阵取单位阵，边上使用 Huber 鲁棒核。
//    因而最小化的是四个角点鲁棒加权的像素重投影误差，而非完整六自由度位姿。
// 4. g2o 使用稠密 Levenberg-Marquardt，初始 lambda 为 0.1，最多迭代
//    20 次。yaw 增量通过 SO(3) 旋转复合后取对数，保持角度的周期性；
//    深度非正或投影点非有限时，该边给出大残差。
// 5. 优化成功后返回 R_world2camera * R_z(yaw*) * R_y(pitch)；
//    角点数量不符、yaw 初值非有限或求解失败时返回原始 R_armor2camera。

BaSolver::BaSolver(const std::array<double, 9> &camera_matrix,
                   const std::vector<double> &dist_coeffs) {
  K_ = Eigen::Matrix3d::Identity();
  K_(0, 0) = camera_matrix[0];
  K_(1, 1) = camera_matrix[4];
  K_(0, 2) = camera_matrix[2];
  K_(1, 2) = camera_matrix[5];
  for (std::size_t i = 0; i < std::min<std::size_t>(dist_coeffs.size(), 5); ++i) {
    distortion_[i] = dist_coeffs[i];
  }

  // Optimization information
  optimizer_.setVerbose(false);
  // Optimization method
  optimizer_.setAlgorithm(
      g2o::OptimizationAlgorithmFactory::instance()->construct(
          "lm_dense", solver_property_));
  // Initial step size
  lm_algorithm_ = dynamic_cast<g2o::OptimizationAlgorithmLevenberg *>(
      const_cast<g2o::OptimizationAlgorithm *>(optimizer_.algorithm()));
  if (lm_algorithm_ == nullptr) {
    throw std::runtime_error("[ba_solver]: failed to construct lm_dense optimizer");
  }
  lm_algorithm_->setUserLambdaInit(0.1);
}

Eigen::Matrix3d
BaSolver::solveBa(const Armor &armor, const std::vector<cv::Point2f> &image_points,
                  const Eigen::Vector3d &t_camera_armor,
                  const Eigen::Matrix3d &R_armor2camera,
                  const Eigen::Matrix3d &R_camera2world,
                  double initial_armor_yaw) noexcept {
  if (image_points.size() != Armor::N_LANDMARKS) {
    tools::logger()->warn("[ba_solver]: expected {} image points, got {}",
                          Armor::N_LANDMARKS, image_points.size());
    return R_armor2camera;
  }

  // Reset optimizer
  optimizer_.clear();

  // OpenCV PnP returns armor -> camera. BA optimizes the armor yaw in the
  // world frame, so both the initial yaw and reprojection include the dynamic
  // camera -> world rotation.
  if (!std::isfinite(initial_armor_yaw)) return R_armor2camera;
  const Sophus::SO3d R_world2camera(R_camera2world.transpose());

  // Get the pitch angle of the armor
  double armor_pitch =
      armor.name == ArmorName::outpost ? -ARMOR_PITCH_RAD : ARMOR_PITCH_RAD;
  Sophus::SO3d R_pitch = Sophus::SO3d::exp(Eigen::Vector3d(0, armor_pitch, 0));

  // Get the 3D points of the armor
  const auto armor_size =
      armor.type == ArmorType::small
          ? Eigen::Vector2d(SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT)
          : Eigen::Vector2d(LARGE_ARMOR_WIDTH, LARGE_ARMOR_HEIGHT);
  const auto object_points =
      Armor::buildObjectPoints<Eigen::Vector3d>(armor_size(0), armor_size(1));

  // Fill the optimizer
  size_t id_counter = 0;

  VertexYaw *v_yaw = new VertexYaw();
  v_yaw->setId(id_counter++);
  v_yaw->setEstimate(initial_armor_yaw);
  optimizer_.addVertex(v_yaw);

  for (size_t i = 0; i < Armor::N_LANDMARKS; i++) {
    g2o::VertexPointXYZ *v_point = new g2o::VertexPointXYZ();
    v_point->setId(id_counter++);
    v_point->setEstimate(Eigen::Vector3d(
        object_points[i].x(), object_points[i].y(), object_points[i].z()));
    v_point->setFixed(true);
    optimizer_.addVertex(v_point);

    EdgeProjection *edge =
        new EdgeProjection(R_world2camera, R_pitch, t_camera_armor, K_, distortion_);
    edge->setId(id_counter++);
    edge->setVertex(0, v_yaw);
    edge->setVertex(1, v_point);
    edge->setMeasurement(Eigen::Vector2d(image_points[i].x, image_points[i].y));
    edge->setInformation(EdgeProjection::InfoMatrixType::Identity());
    edge->setRobustKernel(new g2o::RobustKernelHuber);
    optimizer_.addEdge(edge);
  }

  // Start optimizing
  optimizer_.initializeOptimization();
  const int iterations = optimizer_.optimize(20);

  // Get yaw angle after optimization
  double yaw_optimized = v_yaw->estimate();

  // g2o returns zero when the initial estimate already satisfies the
  // stopping criterion; that is a valid result. Negative means failure.
  if (iterations < 0 || !std::isfinite(yaw_optimized)) {
    tools::logger()->warn("[ba_solver]: yaw optimization failed");
    return R_armor2camera;
  }

  Sophus::SO3d R_yaw = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, yaw_optimized));
  return (R_world2camera * R_yaw * R_pitch).matrix();
}




void VertexYaw::oplusImpl(const double *update) {
  Sophus::SO3d R_yaw = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, update[0])) *
                       Sophus::SO3d::exp(Eigen::Vector3d(0, 0, _estimate));
  _estimate = R_yaw.log()(2);
}

EdgeProjection::EdgeProjection(const Sophus::SO3d &R_world2camera,
                               const Sophus::SO3d &R_pitch,
                               const Eigen::Vector3d &t,
                               const Eigen::Matrix3d &K,
                               const Eigen::Matrix<double, 5, 1> &distortion)
    : R_world2camera_(R_world2camera),
      R_pitch_(R_pitch),
      t_(t),
      K_(K),
      distortion_(distortion) {}

void EdgeProjection::computeError() {
  // Get the rotation
  double yaw = static_cast<VertexYaw *>(_vertices[0])->estimate();
  Sophus::SO3d R_yaw = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, yaw));
  Sophus::SO3d R = R_world2camera_ * R_yaw * R_pitch_;

  // Get the 3D point
  Eigen::Vector3d p_3d =
      static_cast<g2o::VertexPointXYZ *>(_vertices[1])->estimate();

  // Get the observed 2D point
  Eigen::Vector2d obs(_measurement);

  // Project the 3D point to the 2D point
  const Eigen::Vector3d p_camera = R * p_3d + t_;
  if (!p_camera.allFinite() || p_camera.z() <= 1e-9) {
    _error.setConstant(1e6);
    return;
  }

  const double x = p_camera.x() / p_camera.z();
  const double y = p_camera.y() / p_camera.z();
  const double r2 = x * x + y * y;
  const double r4 = r2 * r2;
  const double r6 = r4 * r2;
  const double k1 = distortion_[0];
  const double k2 = distortion_[1];
  const double p1 = distortion_[2];
  const double p2 = distortion_[3];
  const double k3 = distortion_[4];
  const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
  const double x_distorted = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
  const double y_distorted = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
  const Eigen::Vector3d p_2d = K_ * Eigen::Vector3d(x_distorted, y_distorted, 1.0);

  // Calculate the error
  _error = obs - p_2d.head<2>();
}

} // namespace auto_aim
