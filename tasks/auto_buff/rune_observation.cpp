#include "rune_observation.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "tools/logger.hpp"

namespace auto_buff
{
RuneObservationConverter::RuneObservationConverter(const std::string & config_path)
{
  const YAML::Node yaml = YAML::LoadFile(config_path);
  const std::string enemy_color = yaml["enemy_color"].as<std::string>();
  if (enemy_color == "red")
    color_ = RuneInfo::RED;
  else if (enemy_color == "blue")
    color_ = RuneInfo::BLUE;
  else
    throw std::runtime_error("enemy_color must be red or blue");

  const YAML::Node refine = yaml["buff_refine"];
  if (refine) {
    if (refine["extend_rotated_rect"])
      extend_rotated_rect_ = refine["extend_rotated_rect"].as<float>();
    if (refine["visualize_observation"])
      visualize_ = refine["visualize_observation"].as<bool>();
  }
  if (!std::isfinite(extend_rotated_rect_) || extend_rotated_rect_ <= 0.0F)
    throw std::runtime_error("buff_refine.extend_rotated_rect must be greater than zero");
}

int RuneObservationConverter::map_class_for_mode(int model_class, PowerRuneType type)
{
  if (model_class == 0) return 0;
  if (type == PowerRuneType::Small && model_class == 1) return 1;
  if (type == PowerRuneType::Big && model_class == 2) return 1;
  return -1;
}

RuneObservation RuneObservationConverter::convert2rune_observation(
  const cv::Mat & image, const std::vector<YOLO11_BUFF::Object> & detections,
  PowerRuneType type, std::chrono::steady_clock::time_point timestamp) const
{
  RuneObservation observation;
  observation.type = type;
  observation.original_image = image;
  observation.timestamp = timestamp;
  observation.rune_infos.reserve(detections.size());

  const cv::Rect image_rect(0, 0, image.cols, image.rows);
  for (const auto & detection : detections) {
    if (detection.kpt.size() != 5) {
      tools::logger()->warn(
        "[RuneObservationConverter] Expected 5 keypoints, got {}", detection.kpt.size());
      continue;
    }

    const int class_id = map_class_for_mode(detection.label, type);
    if (class_id < 0) continue;

    RuneInfo rune_info;
    rune_info.top = detection.kpt[0];
    rune_info.left = detection.kpt[1];
    rune_info.point_R = detection.kpt[2];
    rune_info.right = detection.kpt[3];
    rune_info.bottom = detection.kpt[4];
    rune_info.class_id = class_id;
    rune_info.confidence = detection.prob;
    rune_info.quality = detection.quality;
    rune_info.color = color_;

    const std::vector<cv::Point2f> points = { // 关键点集
      rune_info.top, rune_info.left, rune_info.point_R, rune_info.right, rune_info.bottom};

    // 拟合椭圆，扩展长短边
    rune_info.rotated_rect = cv::minAreaRect(points);
    rune_info.rotated_rect.size.width *= extend_rotated_rect_;
    rune_info.rotated_rect.size.height *= extend_rotated_rect_;

    // 计算视野矩形，裁剪图像
    rune_info.view_rect = rune_info.rotated_rect.boundingRect() & image_rect;
    if (rune_info.view_rect.empty()) {
      tools::logger()->warn("[RuneObservationConverter] Empty rune view rectangle");
      continue;
    }
    rune_info.view = image(rune_info.view_rect).clone();
    observation.rune_infos.emplace_back(std::move(rune_info));
  }
  return observation;
}

void RuneObservationConverter::visualize_rune_observation(
  cv::Mat & image, const RuneObservation & observation) const
{
  if (!visualize_) return;
  for (const auto & rune_info : observation.rune_infos) {
    cv::rectangle(image, rune_info.view_rect, cv::Scalar(0, 255, 0), 1);
    cv::circle(image, rune_info.top, 3, cv::Scalar(0, 255, 0), 2);
    cv::circle(image, rune_info.left, 3, cv::Scalar(0, 255, 255), 2);
    cv::circle(image, rune_info.point_R, 5, cv::Scalar(255, 0, 0), 2);
    cv::circle(image, rune_info.right, 3, cv::Scalar(255, 0, 255), 2);
    cv::circle(image, rune_info.bottom, 3, cv::Scalar(0, 0, 255), 2);

    const bool inactive = rune_info.class_id == 0;
    const std::string label = inactive ? "UNHIT" : "HIT";
    const cv::Scalar color = inactive ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255);
    cv::putText(
      image, label, rune_info.view_rect.tl() + cv::Point(0, -5), cv::FONT_HERSHEY_SIMPLEX,
      0.5, color, 1, cv::LINE_AA);
  }
}
}  // namespace auto_buff
