#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
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

const std::string keys =
  "{help h usage ? |                  | 输出命令行参数说明}"
  "{@config-path   | configs/uav.yaml | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Camera camera(config_path);
  io::CBoard cboard(config_path);

  auto_aim::Detector detector(config_path);
  auto_aim::Solver solver(config_path);
  // auto_aim::YOLO yolo(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  auto_buff::RuneDetector buff_detector(config_path);
  auto_buff::RuneCamera buff_camera(config_path);
  auto_buff::RuneTargetTracker buff_target_tracker(config_path);
  auto_buff::Aimer buff_aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  while (!exiter.exit()) {
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
    mode = cboard.mode;
    // recorder.record(img, q, t);
    if (last_mode != mode) {
      if (mode == io::Mode::small_buff || mode == io::Mode::big_buff) {
        buff_target_tracker = auto_buff::RuneTargetTracker(config_path);
        buff_aimer = auto_buff::Aimer(config_path);
      }
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    /// 自瞄
    if (mode == io::Mode::auto_aim || mode == io::Mode::outpost) {
      solver.set_R_gimbal2world(q);

      Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

      auto armors = detector.detect(img);

      auto targets = tracker.track(armors, t);

      auto command = aimer.aim(targets, t, cboard.bullet_speed);

      command.shoot = shooter.shoot(command, aimer, targets, ypr);

      cboard.send(command);
    }

    /// 打符
    else if (mode == io::Mode::small_buff || mode == io::Mode::big_buff) {
      buff_camera.set_gimbal_orientation(q);

      const auto camera_pose = buff_camera.pose();
      const auto buff_type =
        mode == io::Mode::small_buff ? auto_buff::PowerRuneType::Small
                                     : auto_buff::PowerRuneType::Big;
      const auto inactive_targets =
        buff_detector.detect(img, buff_type, t, camera_pose, buff_camera);
      auto rune_target =
        buff_target_tracker.track(inactive_targets, img, camera_pose, buff_camera);
      const auto buff_command = buff_aimer.aim(buff_target_tracker, cboard.bullet_speed, true);
      cboard.send(buff_command);
    }

    else
      continue;
  }

  return 0;
}
