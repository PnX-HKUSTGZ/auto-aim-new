#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
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
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

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

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  auto_aim::multithread::MultiThreadDetector detector(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  auto_buff::RuneDetector buff_detector(config_path);
  auto_buff::RuneCamera buff_camera(config_path);
  auto_buff::RuneTargetTracker buff_target_tracker(config_path);
  auto_buff::Aimer buff_aimer(config_path);

  // 自瞄决策线程只在自瞄模式下运行，退出时先等待其停止发送。
  std::unique_ptr<auto_aim::multithread::CommandGener> commandgener;
  std::mutex camera_mutex;

  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE};
  auto last_mode{io::GimbalMode::IDLE};
  auto auto_aim_start = std::chrono::steady_clock::now();
  auto fps_last_time = std::chrono::steady_clock::now();
  int fps_frame_count = 0;

  auto log_fps = [&]() {
    fps_frame_count++;
    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, fps_last_time);
    if (dt >= 1.0) {
      tools::logger()->info("mt_standard FPS: {:.1f}", fps_frame_count / dt);
      fps_frame_count = 0;
      fps_last_time = now;
    }
  };

  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!exiter.exit()) {
      if (mode.load() == io::GimbalMode::AUTO_AIM) {
        {
          std::lock_guard<std::mutex> lock(camera_mutex);
          if (mode.load() != io::GimbalMode::AUTO_AIM) continue;
          camera.read(img, t);
        }
        if (img.empty()) continue;
        detector.push(img, t);
      } else
        std::this_thread::sleep_for(1ms);
    }
  });

  while (!exiter.exit()) {
    const auto current_mode = gimbal.mode();
    mode = current_mode;

    if (last_mode != current_mode) {
      commandgener.reset();
      if (current_mode == io::GimbalMode::AUTO_AIM) {
        auto_aim_start = std::chrono::steady_clock::now();
        commandgener = std::make_unique<auto_aim::multithread::CommandGener>(
          planner, gimbal, plotter);
      } else if (
        current_mode == io::GimbalMode::SMALL_BUFF || current_mode == io::GimbalMode::BIG_BUFF) {
        buff_target_tracker = auto_buff::RuneTargetTracker(config_path);
        buff_aimer = auto_buff::Aimer(config_path);
      }
      tools::logger()->info("Switch to {}", gimbal.str(current_mode));
      last_mode = current_mode;
    }

    /// 自瞄
    if (current_mode == io::GimbalMode::AUTO_AIM) {
      auto [img, armors, t] = detector.debug_pop();
      // 重新进入自瞄时丢弃打符前残留的异步检测结果。
      if (t < auto_aim_start) continue;
      Eigen::Quaterniond q = gimbal.q(t);
      auto gs = gimbal.state();

      // recorder.record(img, q, t);

      solver.set_R_gimbal2world(q);

      auto targets = tracker.track(armors, t);

      commandgener->push(targets, t, gs.bullet_speed);  // 发送给决策线程
      log_fps();

    }

    /// 打符
    else if (
      current_mode == io::GimbalMode::SMALL_BUFF || current_mode == io::GimbalMode::BIG_BUFF) {
      cv::Mat img;
      Eigen::Quaterniond q;
      std::chrono::steady_clock::time_point t;

      {
        std::lock_guard<std::mutex> lock(camera_mutex);
        camera.read(img, t);
      }
      if (img.empty()) {
        gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
        continue;
      }
      q = gimbal.q(t);
      auto gs = gimbal.state();

      // recorder.record(img, q, t);

      buff_camera.set_gimbal_orientation(q);

      const auto camera_pose = buff_camera.pose();
      const auto buff_type =
        current_mode == io::GimbalMode::SMALL_BUFF ? auto_buff::PowerRuneType::Small
                                                   : auto_buff::PowerRuneType::Big;
      const auto inactive_targets =
        buff_detector.detect(img, buff_type, t, camera_pose, buff_camera);
      auto rune_target =
        buff_target_tracker.track(inactive_targets, img, camera_pose, buff_camera);
      // 实机使用采集时间补偿处理延迟。
      const auto buff_plan = buff_aimer.mpc_aim(buff_target_tracker, gs, true);
      gimbal.send(
        buff_plan.control, buff_plan.fire, buff_plan.yaw, buff_plan.yaw_vel, buff_plan.yaw_acc,
        buff_plan.pitch, buff_plan.pitch_vel, buff_plan.pitch_acc);
      log_fps();

    } else {
      gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
      std::this_thread::sleep_for(10ms);
    }
  }

  detect_thread.join();
  commandgener.reset();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}
