// Created by Labor 2023.8.25
// Maintained by Labor, Chengfu Zou
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

#ifndef ARMOR_DETECTOR_BA_SOLVER_HPP_
#define ARMOR_DETECTOR_BA_SOLVER_HPP_

// std
#include <cassert>
#include <array>
#include <cstddef>
#include <tuple>
#include <vector>
// 3rd party
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <sophus/so3.hpp>
// #include <std_msgs/msg/float32.hpp>
// g2o
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
// project
#include "armor.hpp"

namespace auto_aim {

// Custom g2o vertex: Yaw angle (1D optimization variable)
class VertexYaw : public g2o::BaseVertex<1, double> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  VertexYaw() = default;
  virtual void setToOriginImpl() override { _estimate = 0; }
  virtual void oplusImpl(const double *update) override;

  virtual bool read(std::istream &in) override { return true; }
  virtual bool write(std::ostream &out) const override { return true; }
};

// Custom g2o edge: BA reprojection error (2D pixel measurement)
// Connects VertexYaw (vertex 0) and g2o::VertexPointXYZ (vertex 1)
class EdgeProjection : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexYaw,
                                                  g2o::VertexPointXYZ> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using InfoMatrixType = Eigen::Matrix<double, 2, 2>;

  EdgeProjection(const Sophus::SO3d &R_camera_imu, const Sophus::SO3d &R_pitch,
                 const Eigen::Vector3d &t, const Eigen::Matrix3d &K);
  virtual void computeError() override;

  virtual bool read(std::istream &in) override { return true; }
  virtual bool write(std::ostream &out) const override { return true; }

private:
  Sophus::SO3d R_camera_imu_;
  Sophus::SO3d R_pitch_;
  Eigen::Vector3d t_;
  Eigen::Matrix3d K_;
};

// BA algorithm based Optimizer for the armor pose estimation (Particularly for
// the Yaw angle)
class BaSolver {
public:
  BaSolver(std::array<double, 9> &camera_matrix,
           std::vector<double> &dist_coeffs);

  // Solve the armor pose using the BA algorithm, return the optimized rotation
  Eigen::Matrix3d solveBa(const Armor &armor,
                          const Eigen::Vector3d &t_camera_armor,
                          const Eigen::Matrix3d &R_camera_armor,
                          const Eigen::Matrix3d &R_imu_camera) noexcept;

private:
  Eigen::Matrix3d K_;
  g2o::SparseOptimizer optimizer_;
  g2o::OptimizationAlgorithmProperty solver_property_;
  g2o::OptimizationAlgorithmLevenberg *lm_algorithm_;
};

} // namespace auto_aim
#endif // ARMOR_DETECTOR_BAS_SOLVER_HPP_
