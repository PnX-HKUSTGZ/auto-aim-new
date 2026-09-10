#ifndef AUTO_BUFF__RUNE_OBSERVATION_REFINER_HPP
#define AUTO_BUFF__RUNE_OBSERVATION_REFINER_HPP

#include <string>
#include <vector>

#include "rune_observation.hpp"

namespace auto_buff
{
struct RuneRefineConfig
{
  double red_minus_blue_threshold = 50.0;
  double blue_minus_red_threshold = 62.0;
  double armor_area_relative_error_threshold = 0.35;
  int light_arm_line_samples = 15;
  double solidity_threshold_ellipse = 0.8;
  double solidity_threshold_rectangular = 0.66;
  float expected_aspect_ratio = 5.0F;
  float aspect_ratio_relative_error_threshold = 0.42F;
  double approx_error_tolerance = 0.01;
  bool visualize_refined = true;
};

class RuneObservationRefiner
{
public:
  explicit RuneObservationRefiner(const std::string & config_path);

  RefinedRuneObservation refine(const RuneObservation & observation) const;
  void visualize_refined_observation(
    cv::Mat & image, const RefinedRuneObservation & observation) const;

private:
  RuneRefineConfig config_;

  std::vector<std::vector<cv::Point>> extract_contours(const RuneInfo & rune_info) const;
  ConstrainedContours constrain_contours(
    const RuneInfo & rune_info, const std::vector<std::vector<cv::Point>> & contours) const;
  SingleRuneBlade2D construct_blade(
    PowerRuneType type, const RuneInfo & rune_info, ConstrainedContours contours,
    const cv::Rect & image_rect) const;
  bool is_line_pass_through_contour(
    const cv::Point2f & a, const cv::Point2f & b, const std::vector<cv::Point> & contour,
    int samples) const;
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__RUNE_OBSERVATION_REFINER_HPP
