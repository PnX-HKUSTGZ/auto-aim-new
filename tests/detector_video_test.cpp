#include <fmt/core.h>
#include <atomic>
#include <iostream>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "io/gimbal/gimbal.hpp"
#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/sentry.yaml    | yaml配置文件的路径}"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | 0                      | 视频结束帧下标    }"
  "{@video_path    |                        | avi路径}"
  "{tradition t    |  false                 | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  std::cout << "请输入0或1，0为读取视频文件，1为读取摄像头" << std::endl;
  int a;
  std::cin >> a; //a为0则读取视频文件，为1则读取摄像头
  if (a == 0){
    // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto video_path = cli.get<std::string>(0);
  // 如需硬编码路径，取消下面注释并替换路径：
  // video_path = "/home/pnx/sp_vision/sp_vision_25/assets/demo/demo1.avi";
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");
  auto use_tradition = cli.get<bool>("tradition");

  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::VideoCapture video(video_path);

  auto_aim::Detector detector(config_path);
  auto_aim::YOLO yolo(config_path);

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    cv::Mat img;
    std::list<auto_aim::Armor> armors;
    video.read(img);
    if (img.empty()) break;
    // cv::GaussianBlur(img, img, cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);

    if (use_tradition)
      armors = detector.detect(img, frame_count);
    else
      armors = yolo.detect(img, frame_count);

    if (!armors.empty()) {
      nlohmann::json data;
      auto armor = armors.front();

      data["armor_0_pixel_x"] = armor.points[0].x;
      data["armor_0_pixel_y"] = armor.points[0].y;
      data["armor_1_pixel_x"] = armor.points[1].x;
      data["armor_1_pixel_y"] = armor.points[1].y;
      data["armor_2_pixel_x"] = armor.points[2].x;
      data["armor_2_pixel_y"] = armor.points[2].y;
      data["armor_3_pixel_x"] = armor.points[3].x;
      data["armor_3_pixel_y"] = armor.points[3].y;
      plotter.plot(data);
    }

    auto key = cv::waitKey(33);
    if (key == 'q') break;
  }

  return 0;
  } else {
    // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  // auto video_path = cli.get<std::string>(0);
  // auto config_path = cli.get<std::string>("config-path");
  // auto start_index = cli.get<int>("start-index");
  // auto end_index = cli.get<int>("end-index");
  auto use_tradition = cli.get<bool>("tradition");
  auto config_path = cli.get<std::string>(0);

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::Detector detector(config_path);
  auto_aim::YOLO yolo(config_path, true);

  // video.set(cv::CAP_PROP_POS_FRAMES, start_index);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  auto last_reprojection_time = std::chrono::steady_clock::now();
  std::atomic<bool> quit = false;

  while (!exiter.exit()) {
    // if (end_index > 0 && frame_count > end_index) break;

    

    // std::list<auto_aim::Armor> armors;
    camera.read(img, t);
    //if (img.empty()) break;
    // cv::GaussianBlur(img, img, cv::Size(5, 5), 0, 0, cv::BORDER_DEFAULT);

    // if (use_tradition)
    //   armors = detector.detect(img, frame_count);
    // else
    auto armors = yolo.detect(img);

    if (!armors.empty()) {
      nlohmann::json data;
      auto armor = armors.front();

      data["armor_0_pixel_x"] = armor.points[0].x;
      data["armor_0_pixel_y"] = armor.points[0].y;
      data["armor_1_pixel_x"] = armor.points[1].x;
      data["armor_1_pixel_y"] = armor.points[1].y;
      data["armor_2_pixel_x"] = armor.points[2].x;
      data["armor_2_pixel_y"] = armor.points[2].y;
      data["armor_3_pixel_x"] = armor.points[3].x;
      data["armor_3_pixel_y"] = armor.points[3].y;
      plotter.plot(data);
    }

    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last_reprojection_time);
    last_reprojection_time = now;
    tools::draw_text(img, fmt::format("FPS: {:.1f}", 1.0 / dt), {10, 30}, {255, 255, 255});
    cv::imshow("detector_video_test", img);

    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
  }
  
}