#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/rune_camera.hpp"
#include "tasks/auto_buff/rune_detector.hpp"
#include "tasks/auto_buff/rune_tracker.hpp"
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
  "{@input-path    |                        | avi和txt文件的路径}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;
  tools::Exiter exiter;

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  auto_buff::RuneDetector detector(config_path);
  auto_buff::RuneCamera camera(config_path);
  auto_buff::RuneTargetTracker target_tracker(config_path);
  auto_buff::Aimer aimer(config_path);

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();

  io::Command last_command;
  double last_t = -1;

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    /// 自瞄核心逻辑

    camera.set_gimbal_orientation({w, x, y, z});

    const auto camera_pose = camera.pose();
    const auto inactive_targets =
      detector.detect(img, auto_buff::PowerRuneType::Small, timestamp, camera_pose, camera);
    auto rune_target = target_tracker.track(inactive_targets, img, camera_pose, camera, false);
    auto command = aimer.aim(target_tracker, 22, false);

    // cboard.send(command);

    // -------------- 调试输出 --------------

    nlohmann::json data;

    // data["bullet_speed"] = cboard.bullet_speed;

    if (rune_target) {
      const Eigen::Vector3d ypd = tools::xyz2ypd(rune_target->rune_center);
      data["buff_R_yaw"] = ypd[0];
      data["buff_R_pitch"] = ypd[1];
      data["buff_R_dis"] = ypd[2];
      data["phase"] = rune_target->phase * 57.3;
      data["angular_velocity"] = rune_target->angular_velocity;
    }

    // 云台响应情况
    Eigen::Vector3d ypr = tools::eulers(camera.R_gimbal2world(), 2, 1, 0);
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = -ypr[1] * 57.3;

    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
    }

    plotter.plot(data);

    cv::imshow("result", img);

    int key = cv::waitKey(1);
    if (key == 'q') break;
    while (key == ' ') {
      int y = cv::waitKey(30);
      if (y == 'q') break;
    }
  }
  cv::destroyAllWindows();
  text.close();  // 关闭文件

  return 0;
}
