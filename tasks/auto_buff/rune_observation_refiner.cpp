#include "rune_observation_refiner.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "tools/logger.hpp"

namespace auto_buff
{
namespace
{
template<typename T>
void read_optional(const YAML::Node & node, const char * key, T & value)
{
  if (node && node[key]) value = node[key].as<T>();
}

class ContourDescriptor
{
public:
  ContourDescriptor(std::vector<cv::Point> contour, cv::Rect image_rect)
  : contour_(std::move(contour)), image_rect_(image_rect)
  {
  }
  virtual ~ContourDescriptor() = default;
  virtual bool is_usable() const = 0;

protected:
  bool is_near_image_border() const // 判断轮廓是否靠近图像边界，若轮廓中有点距离图像边界小于等于2，则认为靠近
  { 
    for (const auto & point : contour_) {
      const int left = point.x - image_rect_.x;
      const int top = point.y - image_rect_.y;
      const int right = image_rect_.x + image_rect_.width - 1 - point.x;
      const int bottom = image_rect_.y + image_rect_.height - 1 - point.y;
      if (std::min({left, top, right, bottom}) <= 2) return true;
    }
    return false;
  }

  std::vector<cv::Point> contour_;
  cv::Rect image_rect_;
};

double contour_solidity(const std::vector<cv::Point> & contour) // 作用：判断图形是否完整，是否存在破碎情况
{
  const double area = std::abs(cv::contourArea(contour));
  std::vector<cv::Point> hull;
  cv::convexHull(contour, hull); // 计算轮廓的凸包(最小的凸多边形)
  const double hull_area = std::abs(cv::contourArea(hull));
  return hull_area > 1e-6 ? area / hull_area : 0.0;
}

class EllipseDescriptor : public ContourDescriptor
{
public:
  EllipseDescriptor(
    std::vector<cv::Point> contour, const cv::Rect & image_rect, double solidity_threshold)
  : ContourDescriptor(std::move(contour), image_rect), solidity_(contour_solidity(contour_)),
    solidity_threshold_(solidity_threshold), near_border_(is_near_image_border())
  {
  }

  bool is_usable() const override { return solidity_ > solidity_threshold_ && !near_border_; } // 能用需要图像完整且不在边界上

private:
  double solidity_;
  double solidity_threshold_;
  bool near_border_;
};

class RectangularDescriptor : public ContourDescriptor
{
public:
  RectangularDescriptor(
    std::vector<cv::Point> contour, const cv::Rect & image_rect,
    const RuneRefineConfig & config)
  : ContourDescriptor(std::move(contour), image_rect), solidity_(contour_solidity(contour_)),
    config_(config)
  {
    const cv::RotatedRect rect = cv::minAreaRect(contour_);
    const float short_side = std::min(rect.size.width, rect.size.height);
    aspect_ratio_ = short_side > 1e-6F
                      ? std::max(rect.size.width, rect.size.height) / short_side
                      : std::numeric_limits<float>::infinity();
  }

  bool is_usable() const override // 图像是否完整以及矩形长宽比是否符合预期
  {
    if (solidity_ <= config_.solidity_threshold_rectangular) return false;
    const float relative_error =
      std::abs(config_.expected_aspect_ratio - aspect_ratio_) / config_.expected_aspect_ratio;
    return relative_error < config_.aspect_ratio_relative_error_threshold;
  }

private:
  double solidity_;
  float aspect_ratio_ = 0.0F;
  const RuneRefineConfig & config_;
};

class PeakProfileDescriptor : public ContourDescriptor
{
public:
  PeakProfileDescriptor(
    std::vector<cv::Point> contour, const cv::Rect & image_rect, bool is_big_rune,
    double approximation_tolerance)
  : ContourDescriptor(std::move(contour), image_rect), is_big_rune_(is_big_rune)
  {
    const double arc_length = cv::arcLength(contour_, true);
    cv::approxPolyDP(contour_, approximate_contour_, approximation_tolerance * arc_length, true);
    const std::size_t size = approximate_contour_.size();
    valid_size_ = is_big_rune_ ? size >= 8 && size <= 16 : size >= 12 && size <= 16; // 根据拟合顶点数量判断其是否为峰型轮廓，若拟合顶点数量不在范围内，则认为不符合峰型轮廓
    near_border_ = is_big_rune_ && is_near_image_border();
  }

  bool is_usable() const override { return valid_size_ && !near_border_; }

private:
  bool is_big_rune_;
  bool valid_size_ = false;
  bool near_border_ = false;
  std::vector<cv::Point> approximate_contour_;
};
}  // namespace

RuneObservationRefiner::RuneObservationRefiner(const std::string & config_path)
{
  const YAML::Node yaml = YAML::LoadFile(config_path);
  const YAML::Node refine = yaml["buff_refine"];
  read_optional(refine, "red_minus_blue_threshold", config_.red_minus_blue_threshold);
  read_optional(refine, "blue_minus_red_threshold", config_.blue_minus_red_threshold);
  read_optional(
    refine, "armor_area_relative_error_threshold",
    config_.armor_area_relative_error_threshold);
  read_optional(refine, "light_arm_line_samples", config_.light_arm_line_samples);
  read_optional(refine, "solidity_threshold_ellipse", config_.solidity_threshold_ellipse);
  read_optional(
    refine, "solidity_threshold_rectangular", config_.solidity_threshold_rectangular);
  read_optional(refine, "expected_aspect_ratio", config_.expected_aspect_ratio);
  read_optional(
    refine, "aspect_ratio_relative_error_threshold",
    config_.aspect_ratio_relative_error_threshold);
  read_optional(refine, "approx_error_tolerance", config_.approx_error_tolerance);
  read_optional(refine, "visualize_refined", config_.visualize_refined);

  if (config_.light_arm_line_samples < 1)
    throw std::runtime_error("buff_refine.light_arm_line_samples must be positive");
  if (config_.expected_aspect_ratio <= 0.0F)
    throw std::runtime_error("buff_refine.expected_aspect_ratio must be positive");
}

RefinedRuneObservation RuneObservationRefiner::refine(const RuneObservation & observation) const
{
  RefinedRuneObservation refined;
  refined.rune_blades.reserve(observation.rune_infos.size());
  const cv::Rect image_rect(0, 0, observation.original_image.cols, observation.original_image.rows); // 框出图像边界，后续做边界合法性判定
  for (const auto & rune_info : observation.rune_infos) {
    auto contours = extract_contours(rune_info);
    auto constrained = constrain_contours(rune_info, contours); // 对轮廓进行约束，分类得到靶面、灯臂、R标的候选轮廓
    refined.rune_blades.emplace_back(
      construct_blade(observation.type, rune_info, std::move(constrained), image_rect));
  }
  refined.original_image = observation.original_image;
  refined.timestamp = observation.timestamp;
  refined.type = observation.type;
  return refined;
}

std::vector<std::vector<cv::Point>> RuneObservationRefiner::extract_contours( // 用传统cv处理
  const RuneInfo & rune_info) const
{
  if (rune_info.view.empty()) return {};
  std::vector<cv::Mat> bgr;
  cv::split(rune_info.view, bgr);
  cv::Mat difference = rune_info.color == RuneInfo::RED ? bgr[2] - bgr[0] : bgr[0] - bgr[2];
  cv::GaussianBlur(difference, difference, cv::Size(5, 5), 0);
  cv::Mat binary;
  const double threshold = rune_info.color == RuneInfo::RED
                             ? config_.red_minus_blue_threshold
                             : config_.blue_minus_red_threshold;
  cv::threshold(difference, binary, threshold, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  for (auto & contour : contours)
    for (auto & point : contour) point += rune_info.view_rect.tl();
  return contours;
}

ConstrainedContours RuneObservationRefiner::constrain_contours(
  const RuneInfo & rune_info, const std::vector<std::vector<cv::Point>> & contours) const
{
  if (contours.empty()) return {};

  const cv::Point2f armor_center =
    (rune_info.top + rune_info.left + rune_info.bottom + rune_info.right) / 4.0F;
  const double ellipse_area = 0.25 * cv::norm(rune_info.top - rune_info.bottom) *
                              cv::norm(rune_info.left - rune_info.right) * CV_PI;
  if (ellipse_area <= 1e-6) return {};

  const auto is_armor = [&](const std::vector<cv::Point> & contour) { // 是否为靶面：中心是否在轮廓内+面积误差比判定
    const double relative_error =
      std::abs(std::abs(cv::contourArea(contour)) - ellipse_area) / ellipse_area;
    return cv::pointPolygonTest(contour, armor_center, false) > 0 &&
           relative_error <= config_.armor_area_relative_error_threshold;
  };
  const auto is_light_arm = [&](const std::vector<cv::Point> & contour) { // 是否为灯臂：靶心，R标不在轮廓内，两者连线线段过轮廓，armor左右两个点连线线段不过
    if (cv::pointPolygonTest(contour, armor_center, false) >= 0) return false;
    if (cv::pointPolygonTest(contour, rune_info.point_R, false) >= 0) return false;
    if (!is_line_pass_through_contour(
          armor_center, rune_info.point_R, contour, config_.light_arm_line_samples))
      return false;
    return !is_line_pass_through_contour(
      rune_info.left, rune_info.right, contour, config_.light_arm_line_samples);
  };
  const auto is_center_r = [&](const std::vector<cv::Point> & contour) { // 是否为R标
    if (cv::pointPolygonTest(contour, rune_info.point_R, false) <= 0) return false;
    return cv::pointPolygonTest(contour, rune_info.top, false) < 0 &&
           cv::pointPolygonTest(contour, rune_info.left, false) < 0 &&
           cv::pointPolygonTest(contour, rune_info.bottom, false) < 0 &&
           cv::pointPolygonTest(contour, rune_info.right, false) < 0 &&
           cv::pointPolygonTest(contour, armor_center, false) < 0;
  };

  std::vector<std::vector<cv::Point>> armor_candidates;
  std::vector<std::vector<cv::Point>> light_candidates;
  std::vector<std::vector<cv::Point>> center_candidates;
  for (const auto & contour : contours) { // 给不同的轮廓类型分类到对应候选轮廓类型
    if (contour.size() <= 5) continue;
    const bool armor = is_armor(contour);
    const bool light = is_light_arm(contour);
    const bool center = is_center_r(contour);
    if (static_cast<int>(armor) + static_cast<int>(light) + static_cast<int>(center) > 1) {
      tools::logger()->error("[RuneObservationRefiner] Conflicting contour constraints");
      return {};
    }
    if (armor)
      armor_candidates.push_back(contour);
    else if (light)
      light_candidates.push_back(contour);
    else if (center)
      center_candidates.push_back(contour);
  }

  ConstrainedContours result;
  if (!armor_candidates.empty()) {
    const auto best = std::min_element(
      armor_candidates.begin(), armor_candidates.end(), [&](const auto & a, const auto & b) { // 选择最接近椭圆面积的轮廓作为靶面
        return std::abs(std::abs(cv::contourArea(a)) - ellipse_area) <
               std::abs(std::abs(cv::contourArea(b)) - ellipse_area);
      });
    result.armor_module = *best;
  }
  if (!light_candidates.empty()) {
    const auto best = std::max_element(
      light_candidates.begin(), light_candidates.end(), [](const auto & a, const auto & b) { // 选择面积最大的轮廓作为灯臂
        return std::abs(cv::contourArea(a)) < std::abs(cv::contourArea(b));
      });
    result.light_arm = *best;
  }
  if (!center_candidates.empty()) {
    const auto best = std::min_element(
      center_candidates.begin(), center_candidates.end(), [](const auto & a, const auto & b) { // 选择面积最小的轮廓作为R标
        return std::abs(cv::contourArea(a)) < std::abs(cv::contourArea(b));
      });
    result.center_R = *best;
  }
  return result;
}

SingleRuneBlade2D RuneObservationRefiner::construct_blade(
  PowerRuneType type, const RuneInfo & rune_info, ConstrainedContours contours,
  const cv::Rect & image_rect) const
{
  SingleRuneBlade2D blade;
  blade.rune_state = type == PowerRuneType::Big
                       ? (rune_info.class_id == 0 ? RuneState::BigInactive // class_id为0表示未激活，1表示已激活
                                                  : RuneState::BigActivated)
                       : (rune_info.class_id == 0 ? RuneState::SmallInactive
                                                  : RuneState::SmallActivated);
  blade.rune_info = rune_info;
  blade.constrained_contours = std::move(contours);

  if (blade.constrained_contours.armor_module) {
    EllipseDescriptor descriptor( // 椭圆质量检测
      *blade.constrained_contours.armor_module, image_rect,
      config_.solidity_threshold_ellipse);
    blade.is_armor_module_usable = descriptor.is_usable();
  }
  if (blade.constrained_contours.light_arm) {
    if (rune_info.class_id == 0) {
      RectangularDescriptor descriptor( // 矩形质量检测
        *blade.constrained_contours.light_arm, image_rect, config_);
      blade.is_light_arm_usable = descriptor.is_usable();
    } else {
      PeakProfileDescriptor descriptor( // 峰型质量检测
        *blade.constrained_contours.light_arm, image_rect, type == PowerRuneType::Big,
        config_.approx_error_tolerance);
      blade.is_light_arm_usable = descriptor.is_usable();
    }
  }
  blade.is_center_R_usable = blade.constrained_contours.center_R.has_value();
  return blade;
}

bool RuneObservationRefiner::is_line_pass_through_contour( // a到b采样samples个点，判断是否有点在轮廓内
  const cv::Point2f & a, const cv::Point2f & b, const std::vector<cv::Point> & contour,
  int samples) const 
{
  for (int i = 0; i <= samples; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(samples);
    if (cv::pointPolygonTest(contour, a + t * (b - a), false) > 0) return true;
  }
  return false;
}

void RuneObservationRefiner::visualize_refined_observation(
  cv::Mat & image, const RefinedRuneObservation & observation) const
{
  if (!config_.visualize_refined) return;
  for (const auto & blade : observation.rune_blades) {
    const auto draw = [&](const auto & contour, const cv::Scalar & usable_color, bool usable) {
      if (!contour) return;
      const cv::Scalar color = usable ? usable_color : cv::Scalar(80, 80, 80);
      cv::drawContours(image, std::vector<std::vector<cv::Point>>{*contour}, -1, color, 2);
    };
    draw(
      blade.constrained_contours.armor_module, cv::Scalar(0, 255, 0),
      blade.is_armor_module_usable);
    draw(
      blade.constrained_contours.light_arm, cv::Scalar(255, 255, 0),
      blade.is_light_arm_usable);
    draw(
      blade.constrained_contours.center_R, cv::Scalar(0, 0, 255),
      blade.is_center_R_usable);
  }
}
}  // namespace auto_buff
