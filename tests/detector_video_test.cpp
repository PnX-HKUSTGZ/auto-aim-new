#include <fmt/core.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/sentry.yaml    | yaml配置文件的路径}"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | 0                      | 视频结束帧下标    }"
  "{@video_path    |                        | avi路径            }"
  "{tradition t    | false                  | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  std::cout << "请输入0或1，0为读取视频文件，1为读取摄像头" << std::endl;
  int input_mode = 0;
  if (!(std::cin >> input_mode) || (input_mode != 0 && input_mode != 1)) {
    std::cerr << "输入无效，请输入0或1" << std::endl;
    return 1;
  }

  const bool use_camera = input_mode == 1;
  const auto video_path = cli.get<std::string>(0);
  const auto config_path = cli.get<std::string>("config-path");
  const auto start_index = cli.get<int>("start-index");
  const auto end_index = cli.get<int>("end-index");
  const auto use_tradition = cli.get<bool>("tradition");

  tools::Exiter exiter;
  tools::Plotter plotter;

  std::unique_ptr<io::Camera> camera;
  cv::VideoCapture video;
  if (use_camera) {
    camera = std::make_unique<io::Camera>(config_path);
  } else {
    video.open(video_path);
    if (!video.isOpened()) {
      std::cerr << "无法打开视频文件: " << video_path << std::endl;
      return 1;
    }
    video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  }

  auto_aim::Detector detector(config_path);
  auto_aim::YOLO yolo(config_path, use_camera);
  auto last_frame_time = std::chrono::steady_clock::now();

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (!use_camera && end_index > 0 && frame_count > end_index) break;

    cv::Mat img;
    if (use_camera) {
      std::chrono::steady_clock::time_point timestamp;
      camera->read(img, timestamp);
    } else {
      video.read(img);
    }
    if (img.empty()) break;

    auto armors = use_tradition ? detector.detect(img, frame_count) : yolo.detect(img, frame_count);

    if (!armors.empty()) {
      nlohmann::json data;
      const auto & armor = armors.front();
      for (std::size_t i = 0; i < armor.points.size(); i++) {
        data[fmt::format("armor_{}_pixel_x", i)] = armor.points[i].x;
        data[fmt::format("armor_{}_pixel_y", i)] = armor.points[i].y;
      }
      plotter.plot(data);
    }

    if (use_camera) {
      const auto now = std::chrono::steady_clock::now();
      const auto dt = tools::delta_time(now, last_frame_time);
      last_frame_time = now;
      if (dt > 0) {
        tools::draw_text(img, fmt::format("FPS: {:.1f}", 1.0 / dt), {10, 30}, {255, 255, 255});
      }
      cv::imshow("detector_video_test", img);
    }

    const auto key = cv::waitKey(use_camera ? 1 : 33);
    if (key == 'q') break;
  }

  return 0;
}
