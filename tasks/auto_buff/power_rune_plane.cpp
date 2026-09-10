#include "power_rune_plane.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/SVD>
#include <opencv2/core/eigen.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_buff
{
namespace
{
constexpr double kDefaultRuneRadius = 0.7;          // 能量机关中心到装甲中心的默认距离，单位：m
constexpr double kArmorHalfSize = 0.127;            // 装甲左右关键点到装甲中心的半间距，单位：m
constexpr double kExoskeletonHalfSide = 0.24;       // 可视化外骨骼方形截面的半边长，单位：m
constexpr double kExoskeletonHalfThickness = 0.12;  // 可视化外骨骼沿法向的半厚度，单位：m
constexpr int kRuneBladeCount = 5;                  // 能量机关扇叶总数，相邻扇叶间隔 72°

// 能量机关局部模型坐标，单位：m。原点为 R 标中心，-Y 指向当前装甲，Z 为机关平面法向。
// 点序必须与图像关键点 top、left、bottom、right、point_R 的顺序保持一致，用于 PnP。
const std::vector<cv::Point3f> kModelKeypoints = {
  {0.0F, -0.827F, 0.0F},                              // 装甲上关键点
  {-static_cast<float>(kArmorHalfSize), -0.7F, 0.0F},  // 装甲左关键点
  {0.0F, -0.573F, 0.0F},                              // 装甲下关键点
  {static_cast<float>(kArmorHalfSize), -0.7F, 0.0F},   // 装甲右关键点
  {0.0F, 0.0F, 0.0F}};                                // R 标中心（模型原点）

// 局部模型平面上的辅助采样点，单位：m；经 PnP 位姿变换到世界坐标后用于 SVD 拟合机关平面。
const std::array<Eigen::Vector3d, 10> kPlanePoints = {
  Eigen::Vector3d{0.0, 0.0, 0.0},         // R 标中心
  Eigen::Vector3d{0.0, -0.7, 0.0},        // 装甲中心
  Eigen::Vector3d{0.03, -0.17394, 0.0},   // 灯臂靠近 R 标端的右侧参考点
  Eigen::Vector3d{-0.03, -0.17394, 0.0},  // 灯臂靠近 R 标端的左侧参考点
  Eigen::Vector3d{0.03, -0.50394, 0.0},   // 灯臂靠近装甲端的右侧参考点
  Eigen::Vector3d{-0.03, -0.50394, 0.0},  // 灯臂靠近装甲端的左侧参考点
  Eigen::Vector3d{0.0, -0.85, 0.0},       // 装甲区域沿径向远离 R 标的参考点
  Eigen::Vector3d{0.0, -0.55, 0.0},       // 装甲区域沿径向靠近 R 标的参考点
  Eigen::Vector3d{0.15, -0.7, 0.0},       // 装甲区域右侧参考点
  Eigen::Vector3d{-0.15, -0.7, 0.0}};     // 装甲区域左侧参考点

bool finite_point(const cv::Point2f & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

using CubePoints = std::array<Eigen::Vector3d, 8>;

std::array<CubePoints, kRuneBladeCount> build_exoskeleton_points(double rune_radius)
{
  const std::array<Eigen::Vector2d, 4> corners = {
    Eigen::Vector2d{-kExoskeletonHalfSide, kExoskeletonHalfSide},
    Eigen::Vector2d{kExoskeletonHalfSide, kExoskeletonHalfSide},
    Eigen::Vector2d{kExoskeletonHalfSide, -kExoskeletonHalfSide},
    Eigen::Vector2d{-kExoskeletonHalfSide, -kExoskeletonHalfSide}};
  std::array<CubePoints, kRuneBladeCount> cubes;
  for (int cube_index = 0; cube_index < kRuneBladeCount; ++cube_index) {
    const double angle = cube_index * 2.0 * M_PI / kRuneBladeCount;
    const Eigen::Matrix2d rotation =
      tools::rotation_matrix(Eigen::Vector3d(angle, 0.0, 0.0)).topLeftCorner<2, 2>();
    const Eigen::Vector2d center = rotation * Eigen::Vector2d(0.0, -rune_radius);
    std::size_t point_index = 0;
    for (double z : {kExoskeletonHalfThickness, -kExoskeletonHalfThickness}) {
      for (const Eigen::Vector2d & corner : corners) {
        const Eigen::Vector2d rotated = rotation * corner;
        cubes[cube_index][point_index++] =
          Eigen::Vector3d(center.x() + rotated.x(), center.y() + rotated.y(), z);
      }
    }
  }
  return cubes;
}
}  // namespace

PowerRunePlane::PowerRunePlane(const std::string & config_path) : pose_optimizer_(config_path)
{
  const YAML::Node yaml = YAML::LoadFile(config_path);
  if (yaml["buff_phase"] && yaml["buff_phase"]["rune_radius"])
    rune_radius_ = yaml["buff_phase"]["rune_radius"].as<double>();
  if (!std::isfinite(rune_radius_) || rune_radius_ <= 0.0) rune_radius_ = kDefaultRuneRadius;

  const YAML::Node refine = yaml["buff_refine"];
  if (refine && refine["visualize_plane"])
    visualize_ = refine["visualize_plane"].as<bool>();
  else if (refine && refine["visualize_refined"])
    visualize_ = refine["visualize_refined"].as<bool>();
}

bool PowerRunePlane::valid_keypoints(const RuneInfo & info) //info中关键点是否有效
{
  return finite_point(info.top) && finite_point(info.left) && finite_point(info.bottom) &&
         finite_point(info.right) && finite_point(info.point_R);
}

std::optional<PowerRunePlane::ReconstructedPiece> PowerRunePlane::reconstruct_inactive_piece(
  const SingleRuneBlade2D & blade, const CameraPose & camera_pose,
  const RuneCamera & camera) const
{
  const bool inactive = blade.rune_state == RuneState::SmallInactive ||
                        blade.rune_state == RuneState::BigInactive;
  if (!inactive || !valid_keypoints(blade.rune_info)) return std::nullopt;

  const RuneInfo & info = blade.rune_info;
  const std::vector<cv::Point2f> image_points = {
    info.top, info.left, info.bottom, info.right, info.point_R}; // 五点PnP解算

  cv::Vec3d rvec;
  cv::Vec3d tvec;
  const bool solved = cv::solvePnP(
    kModelKeypoints, image_points, camera.camera_matrix(), camera.distort_coeffs(), rvec,
    tvec, false, cv::SOLVEPNP_IPPE);
  if (!solved || !cv::checkRange(rvec) || !cv::checkRange(tvec) || tvec[2] <= 0.0) // 解算失败（非有限数）或解算结果不合法（结果不符合实际）
    return std::nullopt;
  tools::logger()->debug(
    "[PowerRunePlane] PnP solved: rvec[0] = {},rvec[1] = {},rvec[2] = {}, tvec[0] = {}, tvec[1] = {}, tvec[2] = {}", rvec[0], rvec[1], rvec[2], tvec[0], tvec[1], tvec[2]);
  cv::Mat rotation_cv;
  cv::Rodrigues(rvec, rotation_cv);
  Eigen::Matrix3d R_camera_model;
  cv::cv2eigen(rotation_cv, R_camera_model);
  const Eigen::Vector3d t_camera_model(tvec[0], tvec[1], tvec[2]);

  const Eigen::Matrix3d R_world_model = camera_pose.R_world_camera * R_camera_model;
  const Eigen::Vector3d t_world_model =
    camera_pose.R_world_camera * t_camera_model + camera_pose.t_world_camera;

  ReconstructedPiece reconstructed;
  reconstructed.R_world_model = R_world_model;
  reconstructed.plane_points.reserve(kPlanePoints.size());
  for (const Eigen::Vector3d & point : kPlanePoints)
    reconstructed.plane_points.emplace_back(R_world_model * point + t_world_model);

  reconstructed.piece.rune_center = t_world_model;
  reconstructed.piece.armor_center =
    R_world_model * Eigen::Vector3d(0.0, -rune_radius_, 0.0) + t_world_model;
  reconstructed.normal = (R_world_model * Eigen::Vector3d::UnitZ()).normalized();
  return reconstructed;
}

std::optional<PowerRunePlane::Result> PowerRunePlane::update_power_rune_plane(
  const RefinedRuneObservation & refined_observation, const CameraPose & camera_pose,
  const RuneCamera & camera) const
{
  const auto optimized_pose = pose_optimizer_.optimize(refined_observation, camera_pose, camera); // 优化符叶姿态(PnP+chamfer+角点重投影)
  if (!optimized_pose) return std::nullopt;

  cv::Mat rotation_cv;
  cv::Rodrigues(optimized_pose->rvec, rotation_cv);
  Eigen::Matrix3d R_camera_model;
  cv::cv2eigen(rotation_cv, R_camera_model);
  const Eigen::Vector3d t_camera_model(
    optimized_pose->tvec[0], optimized_pose->tvec[1], optimized_pose->tvec[2]);
  const Eigen::Matrix3d R_world_model = camera_pose.R_world_camera * R_camera_model;
  const Eigen::Vector3d t_world_model =
    camera_pose.R_world_camera * t_camera_model + camera_pose.t_world_camera;

  std::vector<ReconstructedPiece> reconstructed;
  reconstructed.reserve(optimized_pose->inactive_locations.size());
  for (const int location : optimized_pose->inactive_locations) {
    // 获得3D坐标下R标，靶面中心的坐标，
    ReconstructedPiece piece;
    piece.R_world_model = R_world_model;
    piece.normal = (R_world_model * Eigen::Vector3d::UnitZ()).normalized();
    piece.piece.rune_center = t_world_model;
    const double angle = (location - 1) * 2.0 * M_PI / kRuneBladeCount;
    const Eigen::Matrix3d sector_rotation =
      tools::rotation_matrix(Eigen::Vector3d(angle, 0.0, 0.0));
    piece.piece.armor_center =
      R_world_model * sector_rotation * Eigen::Vector3d(0.0, -rune_radius_, 0.0) +
      t_world_model;
    piece.plane_points.reserve(kPlanePoints.size());
    for (const Eigen::Vector3d & point : kPlanePoints)
      piece.plane_points.emplace_back(
        R_world_model * sector_rotation * point + t_world_model);
    reconstructed.emplace_back(std::move(piece));
  }
   // 保证备选的扇叶情况符合现实（小符一个大符两个）
  if (reconstructed.empty()) return std::nullopt;
  if (refined_observation.type == PowerRuneType::Small && reconstructed.size() > 1)
    reconstructed.resize(1);
  if (refined_observation.type == PowerRuneType::Big && reconstructed.size() > 2)
    reconstructed.resize(2);

  std::vector<Eigen::Vector3d> all_points;
  for (const auto & piece : reconstructed)
    all_points.insert(all_points.end(), piece.plane_points.begin(), piece.plane_points.end()); // piece点全部堆一起后面用来做SVD拟合平面

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d & point : all_points) centroid += point;
  centroid /= static_cast<double>(all_points.size()); // 获得平均点

  Eigen::MatrixXd centered(all_points.size(), 3);
  for (std::size_t index = 0; index < all_points.size(); ++index)
    centered.row(index) = (all_points[index] - centroid).transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
  Eigen::Vector3d normal = svd.matrixV().col(2).normalized(); // PCA拟合平面法向量，svd的V矩阵的第三列就是最小奇异值对应的特征向量，也就是法向量

  Eigen::Vector3d mean_pose_normal = Eigen::Vector3d::Zero();
  for (const auto & piece : reconstructed) {
    Eigen::Vector3d piece_normal = piece.normal;
    if (mean_pose_normal.squaredNorm() > 0.0 && piece_normal.dot(mean_pose_normal) < 0.0) // dot为点积，对相反方向法向量翻转后相加
      piece_normal = -piece_normal;
    mean_pose_normal += piece_normal;
  }
  if (normal.dot(mean_pose_normal) < 0.0) normal = -normal;

  // Keep the normal sign stable relative to the observation ray. Both quantities are in world.
  if (normal.dot(centroid - camera_pose.t_world_camera) < 0.0) normal = -normal;
  Eigen::Hyperplane<double, 3> plane(normal, centroid);

  InactiveTargets targets;
  targets.camera_model_pose = InactiveTargets::CameraModelPose{
    Eigen::Vector3d(
      optimized_pose->rvec[0], optimized_pose->rvec[1], optimized_pose->rvec[2]),
    t_camera_model};
  targets.capture_timestamp = refined_observation.timestamp;
  targets.is_big_rune = refined_observation.type == PowerRuneType::Big;
  targets.power_rune_plane = plane;
  targets.rune_pieces.reserve(reconstructed.size());
  Eigen::Vector3d shared_rune_center = Eigen::Vector3d::Zero();
  // 将所有扇叶的R标中心投影到拟合平面上，求平均值作为共享的符中心
  for (const auto & piece : reconstructed) 
    shared_rune_center += plane.projection(piece.piece.rune_center); 
  shared_rune_center /= static_cast<double>(reconstructed.size());
  for (const auto & piece : reconstructed) {
    InactiveTargets::RunePiece projected = piece.piece;
    projected.rune_center = shared_rune_center;
    projected.armor_center = plane.projection(projected.armor_center);
    targets.rune_pieces.emplace_back(std::move(projected));
  }
  return Result{std::move(targets), std::move(reconstructed)};
}

void PowerRunePlane::visualize_power_rune_plane(
  cv::Mat & image, const std::vector<ReconstructedPiece> & reconstructed,
  const InactiveTargets & targets, const CameraPose & camera_pose,
  const RuneCamera & camera) const
{
  if (!visualize_ || image.empty() || reconstructed.empty()) return;

  const cv::Scalar model_point_color(0, 255, 0);
  const cv::Scalar rune_center_color(255, 0, 0);
  const cv::Scalar armor_center_color(255, 0, 255);
  const cv::Scalar exoskeleton_color(220, 220, 220);
  const cv::Scalar inactive_cube_color(0, 0, 255);
  const cv::Scalar pose_normal_color(0, 165, 255);
  const cv::Scalar fitted_normal_color(0, 0, 255);
  const cv::Scalar start_vector_color(0, 255, 255);

  for (const ReconstructedPiece & piece : reconstructed) {
    for (std::size_t point_index = 0; point_index < piece.plane_points.size(); ++point_index) {
      const auto pixel = camera.project_world_point(piece.plane_points[point_index], camera_pose);
      if (!pixel) continue;
      const cv::Scalar color = point_index == 0
                                 ? rune_center_color
                                 : (point_index == 1 ? armor_center_color : model_point_color);
      cv::circle(image, *pixel, point_index < 2 ? 4 : 2, color, -1, cv::LINE_AA);
    }

    const auto normal_start = camera.project_world_point(piece.piece.armor_center, camera_pose);
    const auto normal_end =
      camera.project_world_point(piece.piece.armor_center + 0.3 * piece.normal, camera_pose);
    if (normal_start && normal_end)
      cv::arrowedLine(
        image, *normal_start, *normal_end, pose_normal_color, 1, cv::LINE_AA, 0, 0.2);
  }

  const ReconstructedPiece & reference = reconstructed.front();
  const Eigen::Vector3d model_origin_world = targets.rune_pieces.front().rune_center;
  const auto model_cubes = build_exoskeleton_points(rune_radius_);
  std::array<Eigen::Vector3d, kRuneBladeCount> cube_centers_world;
  for (int cube_index = 0; cube_index < kRuneBladeCount; ++cube_index) {
    cube_centers_world[cube_index] = Eigen::Vector3d::Zero();
    for (const Eigen::Vector3d & point : model_cubes[cube_index])
      cube_centers_world[cube_index] +=
        reference.R_world_model * point + model_origin_world;
    cube_centers_world[cube_index] /= model_cubes[cube_index].size();
  }

  std::array<bool, kRuneBladeCount> inactive_cubes{};
  for (const auto & target : targets.rune_pieces) {
    const auto closest = std::min_element(
      cube_centers_world.begin(), cube_centers_world.end(),
      [&](const Eigen::Vector3d & lhs, const Eigen::Vector3d & rhs) {
        return (lhs - target.armor_center).squaredNorm() <
               (rhs - target.armor_center).squaredNorm();
      });
    inactive_cubes[std::distance(cube_centers_world.begin(), closest)] = true;
  }

  constexpr std::array<std::pair<int, int>, 12> cube_edges = {{
    {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
    {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}}};
  std::array<std::optional<cv::Point2f>, kRuneBladeCount> cube_center_pixels;
  for (int cube_index = 0; cube_index < kRuneBladeCount; ++cube_index) {
    std::array<std::optional<cv::Point2f>, 8> pixels;
    for (std::size_t point_index = 0; point_index < model_cubes[cube_index].size(); ++point_index) {
      const Eigen::Vector3d point_world =
        reference.R_world_model * model_cubes[cube_index][point_index] +
        model_origin_world;
      pixels[point_index] = camera.project_world_point(point_world, camera_pose);
    }
    const cv::Scalar color = inactive_cubes[cube_index]
                               ? inactive_cube_color
                               : exoskeleton_color;
    for (const auto & [start, end] : cube_edges)
      if (pixels[start] && pixels[end])
        cv::line(image, *pixels[start], *pixels[end], color, 1, cv::LINE_AA);
    cube_center_pixels[cube_index] =
      camera.project_world_point(cube_centers_world[cube_index], camera_pose);
    if (cube_center_pixels[cube_index])
      cv::circle(image, *cube_center_pixels[cube_index], 3, color, -1, cv::LINE_AA);
  }
  for (int cube_index = 0; cube_index < kRuneBladeCount; ++cube_index) {
    const int next = (cube_index + 1) % kRuneBladeCount;
    if (cube_center_pixels[cube_index] && cube_center_pixels[next])
      cv::line(
        image, *cube_center_pixels[cube_index], *cube_center_pixels[next],
        start_vector_color, 1, cv::LINE_AA);
  }

  const Eigen::Vector3d rune_center = targets.rune_pieces.front().rune_center;
  const Eigen::Vector3d normal = targets.power_rune_plane.normal().normalized();
  Eigen::Vector3d start_vector = normal.cross(Eigen::Vector3d::UnitZ());
  if (start_vector.norm() <= 1e-9) start_vector = normal.cross(Eigen::Vector3d::UnitY());
  if (start_vector.norm() > 1e-9) start_vector.normalize();

  const auto center_pixel = camera.project_world_point(rune_center, camera_pose);
  const auto normal_pixel = camera.project_world_point(rune_center + 0.5 * normal, camera_pose);
  const auto start_pixel = camera.project_world_point(
    rune_center + 0.4 * start_vector, camera_pose);
  if (center_pixel && normal_pixel)
    cv::arrowedLine(image, *center_pixel, *normal_pixel, fitted_normal_color, 2, cv::LINE_AA, 0, 0.2);
  if (center_pixel && start_pixel)
    cv::arrowedLine(image, *center_pixel, *start_pixel, start_vector_color, 2, cv::LINE_AA, 0, 0.2);

  if (center_pixel) {
    const std::string label = cv::format(
      "plane n=(%.2f,%.2f,%.2f)", normal.x(), normal.y(), normal.z());
    cv::putText(
      image, label, *center_pixel + cv::Point2f(8.0F, -8.0F), cv::FONT_HERSHEY_SIMPLEX,
      0.4, fitted_normal_color, 1, cv::LINE_AA);
  }
}
}  // namespace auto_buff
