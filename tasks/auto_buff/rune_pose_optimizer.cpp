#include "rune_pose_optimizer.hpp"

#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <ceres/rotation.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_buff
{
namespace
{
constexpr int kRuneBladeCount = 5;
constexpr double kSectorAngle = 2.0 * M_PI / kRuneBladeCount;

struct RuneBladeCorrespondence
{
  RuneState rune_state = RuneState::SmallInactive;
  int location = 1;
  std::vector<Eigen::Vector3d> model_points;
  std::vector<std::vector<cv::Point>> matched_contours;
  std::vector<cv::Point2f> anchor_points;
  Eigen::Vector2d direction = Eigen::Vector2d::Zero();
};

struct CorrespondenceSet
{
  std::vector<RuneBladeCorrespondence> correspondences;
  cv::Point2f rune_center;
};

struct DistanceField
{
  cv::Mat values;
  cv::Rect rect;
};

template<typename T>
void read_optional(const YAML::Node & node, const char * key, T & value)
{
  if (node && node[key]) value = node[key].as<T>();
}

bool is_inactive(RuneState state)
{
  return state == RuneState::SmallInactive || state == RuneState::BigInactive;
}

bool is_usable_for_projection(const SingleRuneBlade2D & blade) // 判断方式：是否激活和是否语意完整
{
  const auto & contours = blade.constrained_contours;
  if (blade.rune_state == RuneState::BigActivated) {
    return blade.is_center_R_usable && blade.is_light_arm_usable && contours.center_R &&
           contours.light_arm;
  }
  return blade.is_armor_module_usable && blade.is_light_arm_usable &&
         blade.is_center_R_usable && contours.armor_module && contours.light_arm &&
         contours.center_R;
}

cv::Point2f contour_center(const std::vector<cv::Point> & contour)
{
  const cv::Moments moments = cv::moments(contour);
  if (std::abs(moments.m00) > 1e-6)
    return cv::Point2f(
      static_cast<float>(moments.m10 / moments.m00),
      static_cast<float>(moments.m01 / moments.m00));

  cv::Point2f center;
  for (const cv::Point & point : contour) center += cv::Point2f(point);
  if (!contour.empty()) center *= 1.0F / static_cast<float>(contour.size());
  return center;
}

std::optional<Eigen::Vector2d> calculate_direction(const SingleRuneBlade2D & blade)
{
  const auto & contours = blade.constrained_contours;
  if (!contours.center_R) return std::nullopt;
  const cv::Point2f rune_center = contour_center(*contours.center_R);

  cv::Point2f target_center;
  if (blade.rune_state == RuneState::BigActivated) {
    if (!contours.light_arm) return std::nullopt;
    target_center = contour_center(*contours.light_arm);
  } else {
    if (!contours.armor_module || contours.armor_module->size() < 5) return std::nullopt;
    target_center = cv::fitEllipse(*contours.armor_module).center;
  }

  Eigen::Vector2d direction(
    static_cast<double>(target_center.x - rune_center.x),
    static_cast<double>(target_center.y - rune_center.y));
  if (!direction.array().isFinite().all() || direction.norm() <= 1e-6)
    return std::nullopt;
  return direction.normalized();
}

Eigen::Matrix3d sector_rotation(int location)
{
  const double angle = (location - 1) * kSectorAngle;
  return tools::rotation_matrix(Eigen::Vector3d(angle, 0.0, 0.0));
}

void append_segment(
  std::vector<Eigen::Vector3d> & points, const Eigen::Vector3d & first,
  const Eigen::Vector3d & last, int samples)
{
  for (int index = 0; index < samples; ++index) {
    const double ratio = samples == 1 ? 0.0 : static_cast<double>(index) / (samples - 1);
    points.emplace_back(first + ratio * (last - first));
  }
}

std::vector<Eigen::Vector3d> inactive_model_points()
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(64);
  append_segment(points, {-0.03, -0.50394, -0.0111}, {-0.03, -0.17394, -0.0111}, 8);
  append_segment(points, {-0.03, -0.17394, -0.0111}, {0.03, -0.17394, -0.0111}, 3);
  append_segment(points, {0.03, -0.17394, -0.0111}, {0.03, -0.50394, -0.0111}, 8);
  append_segment(points, {0.03, -0.50394, -0.0111}, {-0.03, -0.50394, -0.0111}, 3);
  for (int index = 0; index < 36; ++index) {
    const double angle = 2.0 * M_PI * index / 36.0;
    points.emplace_back(0.154 * std::cos(angle), -0.7 + 0.154 * std::sin(angle), 0.0);
  }
  return points;
}

std::vector<Eigen::Vector3d> small_active_model_points()
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(36);
  for (int index = 0; index < 36; ++index) {
    const double angle = 2.0 * M_PI * index / 36.0;
    points.emplace_back(0.154 * std::cos(angle), -0.7 + 0.154 * std::sin(angle), 0.0);
  }
  return points;
}

std::vector<Eigen::Vector3d> big_active_model_points()
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(64);
  constexpr double z = -0.0111;
  append_segment(points, {0.03, -0.17394, z}, {0.03, -0.306107, z}, 6);
  append_segment(points, {-0.03, -0.17394, z}, {-0.03, -0.306107, z}, 6);
  append_segment(points, {0.057809, -0.17394, z}, {0.158635, -0.312715, z}, 8);
  append_segment(points, {0.068, -0.15394, z}, {0.182776, -0.311916, z}, 9);
  append_segment(points, {-0.057809, -0.17394, z}, {-0.158635, -0.312715, z}, 8);
  append_segment(points, {-0.068, -0.15394, z}, {-0.182776, -0.311916, z}, 9);
  append_segment(points, {-0.068, -0.15394, z}, {0.068, -0.15394, z}, 6);
  return points;
}

std::vector<Eigen::Vector3d> model_points_for_state(RuneState state, int location)
{
  std::vector<Eigen::Vector3d> points;
  if (is_inactive(state))
    points = inactive_model_points();
  else if (state == RuneState::SmallActivated)
    points = small_active_model_points();
  else
    points = big_active_model_points();
  const Eigen::Matrix3d rotation = sector_rotation(location);
  for (Eigen::Vector3d & point : points) point = rotation * point;
  return points;
}

std::vector<cv::Point2f> calculate_anchor_points( // 计算符叶的四个角点
  const SingleRuneBlade2D & blade, const cv::Point2f & rune_center)
{
  const auto & armor = *blade.constrained_contours.armor_module;
  const auto & light = *blade.constrained_contours.light_arm;
  cv::Mat data(static_cast<int>(armor.size() + light.size()), 2, CV_64F);
  int row = 0;
  for (const auto * contour : {&armor, &light}) {
    for (const cv::Point & point : *contour) {
      data.at<double>(row, 0) = point.x;
      data.at<double>(row, 1) = point.y;
      ++row;
    }
  }

  cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
  // 获取pca处理后的平面中心点和主轴向量，主轴向量为归一化的二维向量
  const cv::Point2f center(
    static_cast<float>(pca.mean.at<double>(0, 0)),
    static_cast<float>(pca.mean.at<double>(0, 1)));
  const cv::Point2f axis_x( // 主轴向量
    static_cast<float>(pca.eigenvectors.at<double>(0, 0)),
    static_cast<float>(pca.eigenvectors.at<double>(0, 1)));
  const cv::Point2f axis_y(-axis_x.y, axis_x.x);

  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  for (const auto * contour : {&armor, &light}) {
    for (const cv::Point & point : *contour) { // 将各个点投影到pca处理后平面上
      const cv::Point2f offset = cv::Point2f(point) - center;
      const float x = offset.dot(axis_x);
      const float y = offset.dot(axis_y);
      min_x = std::min(min_x, x); // 沿符叶长度方向的边界
      max_x = std::max(max_x, x);
      min_y = std::min(min_y, y); // 沿符叶宽度方向的边界
      max_y = std::max(max_y, y);
    }
  }

  std::array<cv::Point2f, 4> corners = { // 得到框的四个角
    center + min_x * axis_x + min_y * axis_y,
    center + min_x * axis_x + max_y * axis_y,
    center + max_x * axis_x + max_y * axis_y,
    center + max_x * axis_x + min_y * axis_y};
  std::array<int, 4> order = {0, 1, 2, 3};
  std::stable_sort(order.begin(), order.end(), [&](int lhs, int rhs) { // 按照离R标中心的距离排序，离R标中心近的排在前面
    const cv::Point2f lhs_vector = corners[lhs] - rune_center;
    const cv::Point2f rhs_vector = corners[rhs] - rune_center;
    return lhs_vector.dot(lhs_vector) > rhs_vector.dot(rhs_vector);
  });

  cv::Point2f top_left = corners[order[0]];
  cv::Point2f top_right = corners[order[1]];
  cv::Point2f bottom_left = corners[order[2]];
  cv::Point2f bottom_right = corners[order[3]];
  const cv::Point2f box_center =
    (top_left + top_right + bottom_left + bottom_right) * 0.25F;
  const cv::Point2f toward_rune = rune_center - box_center;
  const auto cross_z = [](const cv::Point2f & first, const cv::Point2f & second) { // 叉乘通过正负确定方向
    return first.x * second.y - first.y * second.x;
  };
  if (cross_z(top_right - top_left, toward_rune) <= 0.0F) std::swap(top_left, top_right);
  if (cross_z(bottom_right - bottom_left, toward_rune) <= 0.0F)
    std::swap(bottom_left, bottom_right);
  return {top_left, bottom_left, bottom_right, top_right};
}

std::vector<cv::Point3f> inactive_anchor_model_points(int location) // 计算符叶的四个角点在模型坐标系下的坐标
{
  const std::array<Eigen::Vector3d, 4> base = {
    Eigen::Vector3d{-0.15, -0.85, 0.0}, Eigen::Vector3d{-0.15, -0.174, 0.0},
    Eigen::Vector3d{0.15, -0.174, 0.0}, Eigen::Vector3d{0.15, -0.85, 0.0}};
  std::vector<cv::Point3f> points;
  points.reserve(base.size());
  const Eigen::Matrix3d rotation = sector_rotation(location);
  for (const Eigen::Vector3d & point : base) {
    const Eigen::Vector3d rotated = rotation * point;
    points.emplace_back(
      static_cast<float>(rotated.x()), static_cast<float>(rotated.y()),
      static_cast<float>(rotated.z()));
  }
  return points;
}

std::optional<int> match_location(
  const Eigen::Vector2d & reference, const Eigen::Vector2d & direction,
  double half_interval)
{
  const double angle = std::atan2(
    reference.x() * direction.y() - reference.y() * direction.x(),
    reference.dot(direction));
  for (int location = 2; location <= kRuneBladeCount; ++location) {
    const double expected = (location - 1) * kSectorAngle;
    if (std::abs(tools::limit_rad(angle - expected)) <= half_interval) return location;
  }
  return std::nullopt;
}

RuneBladeCorrespondence make_correspondence( // 转换，新结构体中进行了pca
  const SingleRuneBlade2D & blade, int location, const cv::Point2f & rune_center,
  const Eigen::Vector2d & direction)
{
  RuneBladeCorrespondence correspondence;
  correspondence.rune_state = blade.rune_state;
  correspondence.location = location;
  correspondence.direction = direction;
  correspondence.model_points = model_points_for_state(blade.rune_state, location);
  if (is_inactive(blade.rune_state)) {
    correspondence.matched_contours.emplace_back(*blade.constrained_contours.armor_module);
    correspondence.matched_contours.emplace_back(*blade.constrained_contours.light_arm);
    correspondence.anchor_points = calculate_anchor_points(blade, rune_center);
  } else if (blade.rune_state == RuneState::SmallActivated) {
    correspondence.matched_contours.emplace_back(*blade.constrained_contours.armor_module);
  } else {
    correspondence.matched_contours.emplace_back(*blade.constrained_contours.light_arm);
  }
  return correspondence;
}

std::optional<CorrespondenceSet> build_correspondences( // 构建符叶对应关系
  const RefinedRuneObservation & observation, double half_interval)
{
  int reference_index = -1;
  for (std::size_t index = 0; index < observation.rune_blades.size(); ++index) {
    if (is_inactive(observation.rune_blades[index].rune_state) &&
        is_usable_for_projection(observation.rune_blades[index])) { // 选取可用的符叶
      reference_index = static_cast<int>(index);
      break;
    }
  }
  if (reference_index < 0) return std::nullopt;

  const SingleRuneBlade2D & reference_blade = observation.rune_blades[reference_index];
  const auto reference_direction = calculate_direction(reference_blade); // 返回归一化二维符叶方向
  if (!reference_direction) return std::nullopt;

  CorrespondenceSet result;
  const auto & center_contour = *reference_blade.constrained_contours.center_R;
  result.rune_center = center_contour.size() >= 5
                         ? cv::fitEllipse(center_contour).center
                         : contour_center(center_contour);
  result.correspondences.emplace_back(make_correspondence(
    reference_blade, 1, result.rune_center, *reference_direction));

  std::array<bool, kRuneBladeCount + 1> occupied{};
  occupied[1] = true;
  for (std::size_t index = 0; index < observation.rune_blades.size(); ++index) {
    if (static_cast<int>(index) == reference_index) continue;
    const SingleRuneBlade2D & blade = observation.rune_blades[index]; // 处理其他符叶
    if (!is_usable_for_projection(blade)) continue;
    if (observation.type == PowerRuneType::Small && is_inactive(blade.rune_state)) continue;
    const auto direction = calculate_direction(blade);
    if (!direction) continue;
    const auto location = match_location(*reference_direction, *direction, half_interval); // 匹配符叶序号
    if (!location || occupied[*location]) continue;
    occupied[*location] = true;
    result.correspondences.emplace_back(
      make_correspondence(blade, *location, result.rune_center, *direction));
  }
  return result;
}

float nonlinear_penalty(float distance) // 非线性惩罚函数，距离越远，惩罚越大
{
  const int layer = std::max(0, static_cast<int>(std::lround(distance)));
  if (layer <= 3) return static_cast<float>(layer);
  if (layer == 4) return 5.0F;
  if (layer == 5) return 8.0F;
  int previous = 5;
  int current = 8;
  for (int index = 6; index <= layer; ++index) {
    const int next = previous + current;
    previous = current;
    current = next;
  }
  return static_cast<float>(current);
}

std::optional<DistanceField> build_distance_field( // 构建距离场
  const std::vector<RuneBladeCorrespondence> & correspondences, const cv::Size & image_size,
  double scale, double max_residual)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::Rect union_rect;
  bool first = true;
  double short_side_sum = 0.0;
  for (const RuneBladeCorrespondence & correspondence : correspondences) {
    for (const auto & contour : correspondence.matched_contours) { // 遍历每个符叶的匹配轮廓
      if (contour.empty()) continue;
      contours.emplace_back(contour);
      const cv::Rect rect = cv::boundingRect(contour);
      short_side_sum += std::min(rect.width, rect.height);
      union_rect = first ? rect : union_rect | rect; // 计算所有轮廓的并集矩形
      first = false;
    }
  }
  if (first || image_size.width <= 0 || image_size.height <= 0) return std::nullopt;

  const cv::Point2f center(
    union_rect.x + 0.5F * union_rect.width, union_rect.y + 0.5F * union_rect.height);
  cv::Rect roi(
    static_cast<int>(std::floor(center.x - 0.5 * union_rect.width * scale)),
    static_cast<int>(std::floor(center.y - 0.5 * union_rect.height * scale)),
    std::max(1, static_cast<int>(std::ceil(union_rect.width * scale))),
    std::max(1, static_cast<int>(std::ceil(union_rect.height * scale))));
  roi &= cv::Rect(0, 0, image_size.width, image_size.height);
  if (roi.width < 2 || roi.height < 2) return std::nullopt;

  cv::Mat binary(roi.size(), CV_8UC1, cv::Scalar(255));
  cv::Mat inside = cv::Mat::zeros(roi.size(), CV_8UC1);
  std::vector<std::vector<cv::Point>> roi_contours;
  roi_contours.reserve(contours.size());
  for (const auto & contour : contours) {
    std::vector<cv::Point> shifted;
    shifted.reserve(contour.size());
    for (const cv::Point & point : contour) { // 将轮廓点转换为roi坐标系下的点
      const cv::Point local = point - roi.tl();
      if (static_cast<unsigned>(local.x) < static_cast<unsigned>(roi.width) &&
          static_cast<unsigned>(local.y) < static_cast<unsigned>(roi.height)) {
        binary.at<unsigned char>(local) = 0;
        shifted.emplace_back(local);
      }
    }
    if (shifted.size() >= 3) roi_contours.emplace_back(std::move(shifted));
  }
  if (!roi_contours.empty())
    cv::drawContours(inside, roi_contours, -1, cv::Scalar(255), cv::FILLED);

  DistanceField result;
  result.rect = roi;
  cv::distanceTransform(binary, result.values, cv::DIST_L2, cv::DIST_MASK_PRECISE);

  int maximum_outer_band = 0;
  while (nonlinear_penalty(maximum_outer_band + 1.0F) <= max_residual) // 计算最大外扩带宽度
    ++maximum_outer_band;
  // 计算roi与union_rect之间的填充距离
  const int roi_padding = std::max(0, std::min({
    union_rect.x - roi.x, union_rect.y - roi.y,
    roi.br().x - union_rect.br().x, roi.br().y - union_rect.br().y}));
  const double mean_short_side = short_side_sum / contours.size();
  const int scale_band = static_cast<int>(
    std::ceil(mean_short_side * std::max(0.0, scale - 1.0) * 0.5));
  const int adaptive_outer_band = // 根据最大外扩带、roi填充距离和缩放带计算适应性外扩带
    std::max(0, std::min({maximum_outer_band, roi_padding, scale_band}));

  for (int y = 0; y < result.values.rows; ++y) {
    float * distances = result.values.ptr<float>(y);
    const unsigned char * inside_row = inside.ptr<unsigned char>(y);
    const unsigned char * edge_row = binary.ptr<unsigned char>(y);
    for (int x = 0; x < result.values.cols; ++x) {
      if (edge_row[x] == 0) {
        distances[x] = 0.0F;
      } else if (inside_row[x] != 0 ||
                 (adaptive_outer_band > 0 && distances[x] <= adaptive_outer_band)) {
        distances[x] = nonlinear_penalty(distances[x]);
      } else {
        distances[x] = static_cast<float>(max_residual);
      }
    }
  }
  if (!result.values.isContinuous()) result.values = result.values.clone();
  return result;
}

class ChamferResidual
{
public:
  ChamferResidual(
    const float * distance_field, cv::Rect distance_rect, Eigen::Vector3d model_point,
    double max_residual, double fx, double fy, double cx, double cy,
    const Eigen::Matrix3d & R_world_camera, double normal_weight)
  : distance_field_(distance_field), distance_rect_(distance_rect),
    model_point_(std::move(model_point)), max_residual_(max_residual), fx_(fx), fy_(fy),
    cx_(cx), cy_(cy), r20_(R_world_camera(2, 0)), r21_(R_world_camera(2, 1)),
    r22_(R_world_camera(2, 2)), normal_weight_(normal_weight)
  {
  }

  template<typename T>
  bool operator()(const T * const pose, T * residual) const
  {
    const T model_point[3] = {
      T(model_point_.x()), T(model_point_.y()), T(model_point_.z())};
    T camera_point[3];
    ceres::AngleAxisRotatePoint(pose, model_point, camera_point);
    camera_point[0] += pose[3];
    camera_point[1] += pose[4];
    camera_point[2] += pose[5];
    if (camera_point[2] <= T(1e-6)) {
      residual[0] = T(max_residual_);
      residual[1] = T(0.0);
      return true;
    }

    const T u = T(fx_) * camera_point[0] / camera_point[2] + T(cx_);
    const T v = T(fy_) * camera_point[1] / camera_point[2] + T(cy_);
    residual[0] = sample(u, v);
    if (residual[0] > T(max_residual_)) residual[0] = T(max_residual_);

    const T model_z[3] = {T(0.0), T(0.0), T(1.0)};
    T camera_z[3];
    ceres::AngleAxisRotatePoint(pose, model_z, camera_z);
    const T normal_world_z =
      T(r20_) * camera_z[0] + T(r21_) * camera_z[1] + T(r22_) * camera_z[2];
    residual[1] = T(normal_weight_) * normal_world_z;
    return true;
  }

private:
  template<typename T>
  static double scalar(const T & value)
  {
    return value;
  }

  template<typename T, int N>
  static double scalar(const ceres::Jet<T, N> & value)
  {
    return value.a;
  }

  template<typename T>
  T discrete(int x, int y) const
  {
    if (x < 0 || x >= distance_rect_.width || y < 0 || y >= distance_rect_.height)
      return T(max_residual_);
    return T(distance_field_[y * distance_rect_.width + x]);
  }

  template<typename T>
  T sample(const T & u, const T & v) const
  {
    const T u0 = ceres::floor(u);
    const T v0 = ceres::floor(v);
    const int x0 = static_cast<int>(scalar(u0)) - distance_rect_.x;
    const int y0 = static_cast<int>(scalar(v0)) - distance_rect_.y;
    const T du = u - u0;
    const T dv = v - v0;
    return (T(1.0) - du) * (T(1.0) - dv) * discrete<T>(x0, y0) +
           du * (T(1.0) - dv) * discrete<T>(x0 + 1, y0) +
           (T(1.0) - du) * dv * discrete<T>(x0, y0 + 1) +
           du * dv * discrete<T>(x0 + 1, y0 + 1);
  }

  const float * distance_field_;
  cv::Rect distance_rect_;
  Eigen::Vector3d model_point_;
  double max_residual_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double r20_;
  double r21_;
  double r22_;
  double normal_weight_;
};

class AnchorReprojectionResidual
{
public:
  AnchorReprojectionResidual(
    Eigen::Vector3d model_point, cv::Point2f observed, double fx, double fy, double cx,
    double cy, double weight)
  : model_point_(std::move(model_point)), observed_(observed), fx_(fx), fy_(fy), cx_(cx),
    cy_(cy), weight_(weight)
  {
  }

  template<typename T>
  bool operator()(const T * const pose, T * residual) const
  {
    const T model_point[3] = {
      T(model_point_.x()), T(model_point_.y()), T(model_point_.z())};
    T camera_point[3];
    ceres::AngleAxisRotatePoint(pose, model_point, camera_point);
    camera_point[0] += pose[3];
    camera_point[1] += pose[4];
    camera_point[2] += pose[5];
    if (camera_point[2] <= T(1e-6)) {
      residual[0] = T(weight_ * 1000.0);
      residual[1] = T(weight_ * 1000.0);
      return true;
    }
    const T u = T(fx_) * camera_point[0] / camera_point[2] + T(cx_);
    const T v = T(fy_) * camera_point[1] / camera_point[2] + T(cy_);
    residual[0] = T(weight_) * (u - T(observed_.x));
    residual[1] = T(weight_) * (v - T(observed_.y));
    return true;
  }

private:
  Eigen::Vector3d model_point_;
  cv::Point2f observed_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double weight_;
};
}  // namespace

RunePoseOptimizer::RunePoseOptimizer(const std::string & config_path)
{
  const YAML::Node yaml = YAML::LoadFile(config_path);
  const YAML::Node config = yaml["buff_pose"] ? yaml["buff_pose"] : yaml["project"];
  read_optional(config, "sample_stride", sample_stride_);
  read_optional(config, "distance_transform_roi_scale", distance_transform_roi_scale_);
  read_optional(config, "anchor_reproj_weight", anchor_reproj_weight_);
  read_optional(config, "max_chamfer_residual", max_chamfer_residual_);
  read_optional(config, "normal_constraint_weight_total", normal_constraint_weight_total_);
  if (config && config["k_axis_y_constraint_weight_total"])
    normal_constraint_weight_total_ = config["k_axis_y_constraint_weight_total"].as<double>();
  read_optional(config, "half_interval", half_interval_);

  sample_stride_ = std::max(1, sample_stride_);
  distance_transform_roi_scale_ = std::max(1.0, distance_transform_roi_scale_);
  anchor_reproj_weight_ = std::max(0.0, anchor_reproj_weight_);
  max_chamfer_residual_ = std::max(1.0, max_chamfer_residual_);
  normal_constraint_weight_total_ = std::max(0.0, normal_constraint_weight_total_);
  half_interval_ = std::clamp(half_interval_, 0.0, 0.5 * kSectorAngle);
}

std::optional<RunePoseOptimizer::Result> RunePoseOptimizer::optimize( // 优化入口
  const RefinedRuneObservation & observation, const CameraPose & camera_pose,
  const RuneCamera & camera) const
{
  const auto correspondence_set = build_correspondences(observation, half_interval_);
  if (!correspondence_set) return std::nullopt;

  std::vector<cv::Point2f> image_anchors; // 符叶四个角点在图像坐标系下的坐标
  std::vector<cv::Point3f> model_anchors; // 符叶四个角点在模型坐标系下的坐标
  std::vector<int> inactive_locations;
  for (const RuneBladeCorrespondence & correspondence : correspondence_set->correspondences) {
    if (!is_inactive(correspondence.rune_state) || correspondence.anchor_points.size() < 4)
      continue;
    const auto anchors = inactive_anchor_model_points(correspondence.location);
    image_anchors.insert(
      image_anchors.end(), correspondence.anchor_points.begin(),
      correspondence.anchor_points.begin() + 4);
    model_anchors.insert(model_anchors.end(), anchors.begin(), anchors.end());
    inactive_locations.emplace_back(correspondence.location);
  }
  if (image_anchors.size() < 4 || model_anchors.size() != image_anchors.size())
    return std::nullopt;

  cv::Vec3d rvec;
  cv::Vec3d tvec;
  if (!cv::solvePnP(
        model_anchors, image_anchors, camera.camera_matrix(), camera.distort_coeffs(), rvec,
        tvec, false, cv::SOLVEPNP_IPPE) ||
      !cv::checkRange(rvec) || !cv::checkRange(tvec) || tvec[2] <= 0.0) {
    return std::nullopt;
  }

  const auto distance_field = build_distance_field(
    correspondence_set->correspondences, observation.original_image.size(),
    distance_transform_roi_scale_, max_chamfer_residual_);
  if (!distance_field) return std::nullopt;

  std::vector<Eigen::Vector3d> sampled_points;
  for (const RuneBladeCorrespondence & correspondence : correspondence_set->correspondences) {
    const auto & model_points = correspondence.model_points;
    for (std::size_t index = 0; index < model_points.size(); index += sample_stride_)
      sampled_points.emplace_back(model_points[index]);
    if (!model_points.empty() && (model_points.size() - 1) % sample_stride_ != 0) // 确保最后一个点也被采样
      sampled_points.emplace_back(model_points.back());
  }
  if (sampled_points.empty()) return std::nullopt;

  std::array<double, 6> pose = {
    rvec[0], rvec[1], rvec[2], tvec[0], tvec[1], tvec[2]};
  const cv::Mat & camera_matrix = camera.camera_matrix();
  const double fx = camera_matrix.at<double>(0, 0);
  const double fy = camera_matrix.at<double>(1, 1);
  const double cx = camera_matrix.at<double>(0, 2);
  const double cy = camera_matrix.at<double>(1, 2);
  const double normal_weight =
    normal_constraint_weight_total_ / std::sqrt(static_cast<double>(sampled_points.size()));

  ceres::Problem problem;
  for (const Eigen::Vector3d & model_point : sampled_points) {
    auto * cost = new ceres::AutoDiffCostFunction<ChamferResidual, 2, 6>(
      new ChamferResidual(
        distance_field->values.ptr<float>(), distance_field->rect, model_point,
        max_chamfer_residual_, fx, fy, cx, cy, camera_pose.R_world_camera,
        normal_weight));
    problem.AddResidualBlock(cost, nullptr, pose.data()); // 添加符叶模型点的chamfer距离误差
  }
  if (anchor_reproj_weight_ > 0.0) {
    for (std::size_t index = 0; index < model_anchors.size(); ++index) {
      const cv::Point3f & anchor = model_anchors[index];
      auto * cost = new ceres::AutoDiffCostFunction<AnchorReprojectionResidual, 2, 6>(
        new AnchorReprojectionResidual(
          Eigen::Vector3d(anchor.x, anchor.y, anchor.z), image_anchors[index], fx, fy, cx,
          cy, anchor_reproj_weight_));
      problem.AddResidualBlock(cost, nullptr, pose.data()); // 添加符叶四个角点的重投影误差
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (!summary.IsSolutionUsable() || !std::all_of(pose.begin(), pose.end(), [](double value) {
        return std::isfinite(value);
      }) || pose[5] <= 0.0) {
    tools::logger()->debug("[RunePoseOptimizer] Ceres pose optimization failed");
    return std::nullopt;
  }

  Result result;
  result.rvec = cv::Vec3d(pose[0], pose[1], pose[2]);
  result.tvec = cv::Vec3d(pose[3], pose[4], pose[5]);
  result.inactive_locations = std::move(inactive_locations);
  return result;
}
}  // namespace auto_buff
