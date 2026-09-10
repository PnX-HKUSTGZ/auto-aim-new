#include <fmt/format.h>

#include <string>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/rune_camera.hpp"
#include "tasks/auto_buff/rune_detector.hpp"
#include "tasks/auto_buff/rune_tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/trajectory.hpp"

// 定义命令行参数
const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // 初始化绘图器、录制器、退出器
  tools::Plotter plotter;
  tools::Recorder recorder;
  tools::Exiter exiter;

  // 初始化云台、相机
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  // 初始化识别器、解算器、追踪器、瞄准器
  auto_buff::RuneDetector detector(config_path);
  auto_buff::RuneCamera camera_model(config_path);
  auto_buff::RuneTargetTracker target_tracker(config_path);
  auto_buff::Aimer aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    q = gimbal.q(t);
    auto gs = gimbal.state();
    // recorder.record(img, q, t);

    // -------------- 打符核心逻辑 --------------

    camera_model.set_gimbal_orientation(q);

    const auto camera_pose = camera_model.pose();
    const auto inactive_targets = detector.detect(
      img, auto_buff::PowerRuneType::Small, t, camera_pose, camera_model);
    auto rune_target = target_tracker.track(
      inactive_targets, img, camera_pose, camera_model);
    auto command = aimer.aim(target_tracker, gs.bullet_speed, true);

    gimbal.send(
      command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);

    // -------------- 调试输出 --------------

    nlohmann::json data;

    if (rune_target) {
      const Eigen::Vector3d ypd = tools::xyz2ypd(rune_target->rune_center);
      data["buff_R_yaw"] = ypd[0];
      data["buff_R_pitch"] = ypd[1];
      data["buff_R_dis"] = ypd[2];
      data["phase"] = rune_target->phase * 57.3;
      data["angular_velocity"] = rune_target->angular_velocity;
      data["inactive_target_num"] = rune_target->inactive_target_num;
    }

    // 云台响应情况
    Eigen::Vector3d ypr = tools::eulers(camera_model.R_gimbal2world(), 2, 1, 0);
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = ypr[1] * 57.3;

    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
      data["shoot"] = command.shoot ? 1 : 0;
    }

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("result", img);

    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}
