#ifndef AUTO_BUFF__RUNE_OBSERVATION_HPP
#define AUTO_BUFF__RUNE_OBSERVATION_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

#include "yolo11_buff.hpp"

namespace auto_buff
{
enum class PowerRuneType { Small, Big };

struct RuneInfo
{
  enum Color { RED = 0, BLUE = 1 };

  cv::Point2f top;
  cv::Point2f left;
  cv::Point2f point_R;
  cv::Point2f right;
  cv::Point2f bottom;
  int class_id = -1;  // 0: inactive, 1: activated in the current rune mode
  float confidence = 0.0F;
  float quality = 0.0F;
  Color color = RED;
  cv::RotatedRect rotated_rect;
  cv::Rect view_rect;
  cv::Mat view;
};

struct RuneObservation
{
  PowerRuneType type = PowerRuneType::Small;
  std::vector<RuneInfo> rune_infos;
  cv::Mat original_image;
  std::chrono::steady_clock::time_point timestamp;
};

struct ConstrainedContours
{
  std::optional<std::vector<cv::Point>> armor_module;
  std::optional<std::vector<cv::Point>> light_arm;
  std::optional<std::vector<cv::Point>> center_R;
};

enum class RuneState { BigInactive, BigActivated, SmallInactive, SmallActivated };

struct SingleRuneBlade2D
{
  RuneInfo rune_info;
  RuneState rune_state = RuneState::SmallInactive;
  ConstrainedContours constrained_contours;
  bool is_armor_module_usable = false;
  bool is_light_arm_usable = false;
  bool is_center_R_usable = false;
};

struct RefinedRuneObservation
{
  std::vector<SingleRuneBlade2D> rune_blades;
  cv::Mat original_image;
  std::chrono::steady_clock::time_point timestamp;
  PowerRuneType type = PowerRuneType::Small;
};

class RuneObservationConverter
{
public:
  explicit RuneObservationConverter(const std::string & config_path);

  RuneObservation convert2rune_observation(
    const cv::Mat & image, const std::vector<YOLO11_BUFF::Object> & detections,
    PowerRuneType type, std::chrono::steady_clock::time_point timestamp) const;

  void visualize_rune_observation(cv::Mat & image, const RuneObservation & observation) const;

private:
  float extend_rotated_rect_ = 1.4F;
  RuneInfo::Color color_ = RuneInfo::RED;
  bool visualize_ = true;

  static int map_class_for_mode(int model_class, PowerRuneType type);
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__RUNE_OBSERVATION_HPP
