#include "yolo11_buff.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <stdexcept>

#include <openvino/core/preprocess/pre_post_process.hpp>

const double ConfidenceThreshold = 0.7f;
const double IouThreshold = 0.4f;
namespace auto_buff
{
namespace
{
template<typename T>
void read_optional(const YAML::Node & yaml, const char * key, T & value)
{
  const YAML::Node node = yaml[key];
  if (node) value = node.as<T>();
}
}  // namespace

YOLO11_BUFF::YOLO11_BUFF(const std::string & config)
{
  const YAML::Node yaml = YAML::LoadFile(config);
  const std::string model_path = yaml["model"].as<std::string>();
  read_optional(yaml, "buff_confidence_threshold", confidence_threshold_);
  read_optional(yaml, "buff_keypoint_confidence_threshold", keypoint_confidence_threshold_);
  read_optional(yaml, "buff_nms_distance", nms_distance_);
  read_optional(yaml, "buff_minimum_valid_keypoints", minimum_valid_keypoints_);

  if (!std::isfinite(confidence_threshold_) || confidence_threshold_ < 0.0F ||
      confidence_threshold_ > 1.0F)
    throw std::runtime_error("buff_confidence_threshold must be in [0, 1]");
  if (!std::isfinite(keypoint_confidence_threshold_) ||
      keypoint_confidence_threshold_ < 0.0F || keypoint_confidence_threshold_ > 1.0F)
    throw std::runtime_error("buff_keypoint_confidence_threshold must be in [0, 1]");
  if (!std::isfinite(nms_distance_) || nms_distance_ <= 0.0F)
    throw std::runtime_error("buff_nms_distance must be greater than zero");
  if (minimum_valid_keypoints_ < 1 || minimum_valid_keypoints_ > NUM_POINTS)
    throw std::runtime_error("buff_minimum_valid_keypoints must be in [1, 5]");

  model = core.read_model(model_path);
  if (model->inputs().size() != 1 || model->outputs().size() != 1)
    throw std::runtime_error("buff model must have exactly one input and one output");

  const ov::Shape input_shape = model->input().get_shape();
  if (input_shape.size() != 4 || input_shape[0] != 1 || input_shape[1] != 3)
    throw std::runtime_error("buff model input must have static NCHW shape [1,3,H,W]");
  input_height_ = static_cast<int>(input_shape[2]);
  input_width_ = static_cast<int>(input_shape[3]);
  if (input_height_ <= 0 || input_width_ <= 0)
    throw std::runtime_error("buff model input height and width must be non-zero");

  // 与 NNDetector 一致：外部居中 letterbox，OpenVINO 负责 BGR u8 NHWC -> RGB f32 NCHW。
  ov::preprocess::PrePostProcessor preprocessor(model);
  preprocessor.input().tensor()
    .set_element_type(ov::element::u8)
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);
  preprocessor.input().preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0F);
  preprocessor.input().model().set_layout("NCHW");
  model = preprocessor.build();

  compiled_model = core.compile_model(model, "CPU");
  infer_request = compiled_model.create_infer_request();
  input_tensor = ov::Tensor(
    ov::element::u8,
    {1, static_cast<std::size_t>(input_height_), static_cast<std::size_t>(input_width_), 3});
  infer_request.set_input_tensor(input_tensor);

  const auto output_port = compiled_model.output();
  const ov::Shape output_shape = output_port.get_shape();
  if (output_shape.size() != 3 || output_shape[0] != 1)
    throw std::runtime_error("buff model output must be [1,C,A] or [1,A,C]");
  if (output_port.get_element_type() != ov::element::f32)
    throw std::runtime_error("buff model output element type must be f32");

  if (output_shape[1] <= 256 && output_shape[2] >= 100) {
    output_is_nca_ = true;
    output_channels_ = static_cast<int>(output_shape[1]);
    anchor_count_ = static_cast<int>(output_shape[2]);
  } else {
    output_is_nca_ = false;
    anchor_count_ = static_cast<int>(output_shape[1]);
    output_channels_ = static_cast<int>(output_shape[2]);
  }

  if (output_channels_ != OUTPUT_CHANNELS) {
    std::ostringstream message;
    message << "unsupported buff head: expected " << OUTPUT_CHANNELS
            << " channels (3 classes + 5x3 keypoints), got " << output_channels_;
    throw std::runtime_error(message.str());
  }

  tools::logger()->info(
    "[YOLO11_BUFF] model={}, input={}x{}, output_layout={}, anchors={}, conf={}, kconf={}, nms={}",
    model_path, input_width_, input_height_, output_is_nca_ ? "[N,C,A]" : "[N,A,C]",
    anchor_count_, confidence_threshold_, keypoint_confidence_threshold_, nms_distance_);
}

cv::Mat YOLO11_BUFF::normalize_input(const cv::Mat & image) const
{
  if (image.empty()) throw std::invalid_argument("cannot infer an empty image");
  if (image.type() == CV_8UC3) return image;

  cv::Mat bgr;
  if (image.type() == CV_8UC1)
    cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
  else if (image.type() == CV_8UC4)
    cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
  else
    throw std::invalid_argument("buff detector expects an 8-bit 1-, 3-, or 4-channel image");
  return bgr;
}

YOLO11_BUFF::LetterboxResult YOLO11_BUFF::preprocess_letterbox(const cv::Mat & image) const // 图像适配YOLO模型
{
  LetterboxResult result;
  result.scale = std::min(
    static_cast<float>(input_width_) / static_cast<float>(image.cols),
    static_cast<float>(input_height_) / static_cast<float>(image.rows));

  const int resized_width = static_cast<int>(std::round(image.cols * result.scale));
  const int resized_height = static_cast<int>(std::round(image.rows * result.scale));
  result.padding_x = (input_width_ - resized_width) / 2;
  result.padding_y = (input_height_ - resized_height) / 2;
  const int right_padding = input_width_ - resized_width - result.padding_x;
  const int bottom_padding = input_height_ - resized_height - result.padding_y;

  cv::Mat resized;
  if (resized_width != image.cols || resized_height != image.rows)
    cv::resize(image, resized, cv::Size(resized_width, resized_height));
  else
    resized = image;

  if (result.padding_x > 0 || result.padding_y > 0 || right_padding > 0 || bottom_padding > 0) {
    cv::copyMakeBorder(
      resized, result.image, result.padding_y, bottom_padding, result.padding_x, right_padding,
      cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  } else {
    result.image = resized;
  }
  return result;
}

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::infer(const cv::Mat & image) // 获得神经网络处理结果
{
  const cv::Mat bgr = normalize_input(image);
  LetterboxResult letterbox = preprocess_letterbox(bgr);
  if (!letterbox.image.isContinuous()) letterbox.image = letterbox.image.clone();

  const std::size_t image_bytes = letterbox.image.total() * letterbox.image.elemSize();
  if (image_bytes != input_tensor.get_byte_size())
    throw std::runtime_error("letterbox image size does not match OpenVINO input tensor");
  std::memcpy(input_tensor.data<std::uint8_t>(), letterbox.image.data, image_bytes);
  infer_request.infer(); // 推理环节
  return postprocess(letterbox, image.cols, image.rows);
}

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::postprocess(
  const LetterboxResult & letterbox, int original_width, int original_height)
{
  const ov::Tensor output_tensor = infer_request.get_output_tensor();
  const float * const output = output_tensor.data<const float>();
  const auto value_at = [this, output](int channel, int anchor) {
    return output_is_nca_ ? output[channel * anchor_count_ + anchor]
                          : output[anchor * output_channels_ + channel];
  };

  std::vector<Object> objects;
  objects.reserve(64);
  for (int anchor = 0; anchor < anchor_count_; ++anchor) { // 筛选出置信度最高的class
    int best_class = -1;
    float best_confidence = 0.0F;
    for (int class_index = 0; class_index < NUM_CLASSES; ++class_index) {
      const float score = value_at(class_index, anchor);
      if (std::isfinite(score) && score > best_confidence) {
        best_confidence = score;
        best_class = class_index;
      }
    }
    if (best_class < 0 || best_confidence < confidence_threshold_) continue;

    Object object;
    object.label = best_class;  // 暂时只保存，下游尚不使用 class_id。
    object.prob = best_confidence;
    object.kpt.reserve(NUM_POINTS);
    object.kpt_confidences.reserve(NUM_POINTS);

    int valid_keypoints = 0;
    float keypoint_confidence_sum = 0.0F;
    cv::Point2f valid_center{0.0F, 0.0F};
    bool invalid_keypoint = false;
    for (int keypoint = 0; keypoint < NUM_POINTS; ++keypoint) {
      const int base = NUM_CLASSES + keypoint * KEYPOINT_SIZE;
      float x = (value_at(base, anchor) - static_cast<float>(letterbox.padding_x)) /
                letterbox.scale;
      float y = (value_at(base + 1, anchor) - static_cast<float>(letterbox.padding_y)) /
                letterbox.scale;
      const float keypoint_confidence = value_at(base + 2, anchor);

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(keypoint_confidence) ||
          x < 0.0F || y < 0.0F) {
        invalid_keypoint = true;
        break;
      }

      x = std::clamp(x, 0.0F, static_cast<float>(original_width - 1));
      y = std::clamp(y, 0.0F, static_cast<float>(original_height - 1));
      object.kpt.emplace_back(x, y);
      object.kpt_confidences.push_back(keypoint_confidence);
      if (keypoint_confidence >= keypoint_confidence_threshold_) {
        ++valid_keypoints;
        keypoint_confidence_sum += keypoint_confidence;
        valid_center += object.kpt.back();
      }
    }

    if (invalid_keypoint || valid_keypoints < minimum_valid_keypoints_) continue;
    object.center = valid_center * (1.0F / static_cast<float>(valid_keypoints));
    object.quality = object.prob * keypoint_confidence_sum / static_cast<float>(valid_keypoints);
    object.rect = cv::boundingRect(object.kpt);
    objects.emplace_back(std::move(object));
  }
  return center_distance_nms(std::move(objects));
}

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::center_distance_nms( // 计算NMS值
  std::vector<Object> objects) const
{
  std::sort(objects.begin(), objects.end(), [](const Object & left, const Object & right) {
    return left.quality > right.quality;
  });

  std::vector<Object> kept;
  kept.reserve(objects.size());
  std::vector<bool> suppressed(objects.size(), false);
  const float threshold_squared = nms_distance_ * nms_distance_;
  for (std::size_t i = 0; i < objects.size(); ++i) {
    if (suppressed[i]) continue;
    kept.push_back(objects[i]);
    for (std::size_t j = i + 1; j < objects.size(); ++j) {
      if (suppressed[j]) continue;
      const cv::Point2f delta = objects[i].center - objects[j].center;
      if (delta.dot(delta) < threshold_squared) suppressed[j] = true;
    }
  }
  return kept;
}

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::get_multicandidateboxes(cv::Mat & image)
{
  if (image.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return {};
  }
  std::vector<Object> objects = infer(image);
  return objects;
}

}  // namespace auto_buff
