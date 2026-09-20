#include <fmt/format.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/rune_camera.hpp"
#include "tasks/auto_buff/rune_detector.hpp"
#include "tasks/auto_buff/rune_tracker.hpp"
#include "tasks/auto_buff/rune_scene.hpp"
#include "tools/foxglove_server.hpp"
#include "foxglove_scene_schema.hpp"
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
  "{@config-path   | | yaml配置文件路径 }"
  "{foxglove      |true| 启用 Foxglove 三维可视化 }"
  "{foxglove-host |127.0.0.1| WebSocket 监听 IP，远程连接用 0.0.0.0 }"
  "{foxglove-port |8765| WebSocket 监听端口 }"
  "{foxglove-prediction |0.2| 三维运动预览时长，单位秒 }";


int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  const bool foxglove_enabled = cli.get<bool>("foxglove");
  const auto foxglove_host = cli.get<std::string>("foxglove-host");
  const int foxglove_port = cli.get<int>("foxglove-port");
  const double preview_seconds = cli.get<double>("foxglove-prediction");
  if (!cli.check()) {
    cli.printErrors();
    return 1;
  }
  if (foxglove_enabled && (foxglove_port < 1 || foxglove_port > 65535 ||
      !std::isfinite(preview_seconds) || preview_seconds <= 0 || preview_seconds > 10)) {
    tools::logger()->error("Foxglove port must be 1..65535; prediction must be in (0, 10] seconds");
    return 1;
  }
  std::unique_ptr<tools::FoxgloveServer> foxglove;
  if (foxglove_enabled) {
    try {
      foxglove = std::make_unique<tools::FoxgloveServer>(
        foxglove_host, static_cast<std::uint16_t>(foxglove_port),
        std::vector<tools::FoxgloveServer::Channel>{
          {1, "/buff/scene", "foxglove.SceneUpdate", tools::kFoxgloveSceneSchema},
          {2, "/buff/transforms", "foxglove.FrameTransform", tools::kFoxgloveTransformSchema},
          {3, "/buff/telemetry", "auto_buff.Telemetry", tools::kBuffTelemetrySchema}});
      tools::logger()->info(
        "Foxglove: ws://{}:{}, topics /buff/scene + /buff/transforms + /buff/telemetry",
        foxglove_host, foxglove_port);
    } catch (const std::exception & e) {
      tools::logger()->error("Cannot start Foxglove: {}. Use --foxglove=false to disable it.", e.what());
      return 1;
    }
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
    auto rune_target = target_tracker.track(inactive_targets, img, camera_pose, camera_model);
    // 实机使用采集时间补偿处理延迟。
    auto plan = aimer.mpc_aim(target_tracker, gs, true);

    gimbal.send(
      plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
      plan.pitch_acc);
    // -------------- 调试输出 --------------

    nlohmann::json data =
      auto_buff::make_rune_telemetry(inactive_targets, rune_target, preview_seconds);
    data["bullet_speed_m_s"] = gs.bullet_speed;
    data["plan_control"] = plan.control ? 1 : 0;
    data["plan_fire"] = plan.fire ? 1 : 0;
    data["plan_yaw_rad"] = plan.yaw;
    data["plan_pitch_rad"] = plan.pitch;
    data["plan_yaw_velocity_rad_s"] = plan.yaw_vel;
    data["plan_pitch_velocity_rad_s"] = plan.pitch_vel;
    data["plan_yaw_acceleration_rad_s2"] = plan.yaw_acc;
    data["plan_pitch_acceleration_rad_s2"] = plan.pitch_acc;

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
    data["gimbal_yaw"] = gs.yaw * 57.3;
    data["gimbal_pitch"] = gs.pitch * 57.3;
    data["gimbal_yaw_vel"] = gs.yaw_vel * 57.3;
    data["gimbal_pitch_vel"] = gs.pitch_vel * 57.3;

    if (plan.control) {
      data["plan_yaw"] = plan.yaw * 57.3;
      data["plan_pitch"] = plan.pitch * 57.3;
      data["plan_yaw_vel"] = plan.yaw_vel * 57.3;
      data["plan_pitch_vel"] = plan.pitch_vel * 57.3;
      data["plan_yaw_acc"] = plan.yaw_acc * 57.3;
      data["plan_pitch_acc"] = plan.pitch_acc * 57.3;
      data["shoot"] = plan.fire ? 1 : 0;
    }

    plotter.plot(data);
    if (foxglove) {
      const auto timestamp_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());
      auto scene = auto_buff::make_rune_scene(
        inactive_targets, rune_target, camera_pose, timestamp_ns, preview_seconds);
      foxglove->publish(timestamp_ns, {
        {2, auto_buff::make_camera_transform(camera_pose, timestamp_ns).dump()},
        {1, scene.dump()},
        {3, data.dump()}});
    }

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("result", img);

    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}
