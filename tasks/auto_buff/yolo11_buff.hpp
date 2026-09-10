#ifndef AUTO_BUFF__YOLO11_BUFF_HPP
#define AUTO_BUFF__YOLO11_BUFF_HPP
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "tools/logger.hpp"

namespace auto_buff
{
class YOLO11_BUFF
{
public:
  struct Object
  {
    cv::Rect_<float> rect;
    int label = -1;
    float prob = 0.0F;
    float quality = 0.0F;
    cv::Point2f center{0.0F, 0.0F};
    std::vector<cv::Point2f> kpt;
    std::vector<float> kpt_confidences;
  };

  explicit YOLO11_BUFF(const std::string & config);

  // 使用NMS，用来获取多个框
  std::vector<Object> get_multicandidateboxes(cv::Mat & image);

private:
  struct LetterboxResult
  {
    cv::Mat image;
    float scale = 1.0F;
    int padding_x = 0;
    int padding_y = 0;
  };

  ov::Core core;  // 创建OpenVINO Runtime Core对象
  std::shared_ptr<ov::Model> model;
  ov::CompiledModel compiled_model;
  ov::InferRequest infer_request;
  ov::Tensor input_tensor;

  static constexpr int NUM_CLASSES = 3;
  static constexpr int NUM_POINTS = 5;
  static constexpr int KEYPOINT_SIZE = 3;
  static constexpr int OUTPUT_CHANNELS = NUM_CLASSES + NUM_POINTS * KEYPOINT_SIZE;

  int input_width_ = 0;
  int input_height_ = 0;
  int output_channels_ = 0;
  int anchor_count_ = 0;
  bool output_is_nca_ = true;

  float confidence_threshold_ = 0.8F;
  float keypoint_confidence_threshold_ = 0.8F;
  float nms_distance_ = 30.0F;
  int minimum_valid_keypoints_ = NUM_POINTS;

  cv::Mat normalize_input(const cv::Mat & image) const;
  LetterboxResult preprocess_letterbox(const cv::Mat & image) const;
  std::vector<Object> infer(const cv::Mat & image);
  std::vector<Object> postprocess(
    const LetterboxResult & letterbox, int original_width, int original_height);
  std::vector<Object> center_distance_nms(std::vector<Object> objects) const;
};
}  // namespace auto_buff
#endif
