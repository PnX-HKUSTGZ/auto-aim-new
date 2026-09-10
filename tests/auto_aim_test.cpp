#include <fmt/core.h>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "foxglove_scene_schema.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/auto_aim_scene.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/foxglove_server.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/standard3.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{foxglove       | true              | 启用 Foxglove 三维可视化 }"
  "{foxglove-host  | 127.0.0.1         | Foxglove WebSocket 监听 IP }"
  "{foxglove-port  | 8765              | Foxglove WebSocket 监听端口 }"
  "{@input-path    | assets/demo/demo   | avi和txt文件的路径（不含扩展名）}";

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
  const bool foxglove_enabled = cli.get<bool>("foxglove");
  const std::string foxglove_host = cli.get<std::string>("foxglove-host");
  const int foxglove_port = cli.get<int>("foxglove-port");
  if (!cli.check()) {
    cli.printErrors();
    return 1;
  }
  if (foxglove_enabled && (foxglove_port < 1 || foxglove_port > 65535)) {
    tools::logger()->error("Foxglove port must be in [1, 65535]");
    return 1;
  }

  std::unique_ptr<tools::FoxgloveServer> foxglove;
  if (foxglove_enabled) {
    try {
      foxglove = std::make_unique<tools::FoxgloveServer>(
        foxglove_host, static_cast<std::uint16_t>(foxglove_port),
        std::vector<tools::FoxgloveServer::Channel>{
          {1, "/auto_aim/targets", "foxglove.SceneUpdate", tools::kFoxgloveSceneSchema},
          {2, "/auto_aim/transforms", "foxglove.FrameTransform",
           tools::kFoxgloveTransformSchema}});
      tools::logger()->info(
        "Foxglove: ws://{}:{}, topics /auto_aim/targets + /auto_aim/transforms, frame world",
        foxglove_host, foxglove_port);
    } catch (const std::exception & error) {
      tools::logger()->error("Cannot start Foxglove: {}", error.what());
      return 1;
    }
  }

  tools::Plotter plotter;
  tools::Exiter exiter;

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);
  if (!video.isOpened() || !text.is_open()) {
    tools::logger()->error("Cannot open replay input: {}.avi + {}.txt", input_path, input_path);
    return 1;
  }

  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::AutoAimVisualizer auto_aim_visualizer(config_path);
  constexpr double bullet_speed = 27.0;

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();
  auto last_reprojection_time = std::chrono::steady_clock::now();

  auto_aim::Target last_target;
  io::Command last_command;
  double last_t = -1;

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  bool input_exhausted = false;
  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    if (!video.read(img) || img.empty()) {
      input_exhausted = true;
      break;
    }

    double t, w, x, y, z;
    if (!(text >> t >> w >> x >> y >> z)) {
      input_exhausted = true;
      break;
    }
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    /// 自瞄核心逻辑

    solver.set_R_gimbal2world({w, x, y, z});

    auto yolo_start = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, frame_count);

    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, timestamp);

    auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, timestamp, bullet_speed, false);

    if (
      !targets.empty() && aimer.debug_aim_point.valid &&
      std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
      command.shoot = true;

    if (command.control) last_command = command;

    if (foxglove) {
      const auto timestamp_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());
      const Eigen::Quaterniond attitude(solver.R_gimbal2world());
      const double invalid = std::numeric_limits<double>::quiet_NaN();
      const auto scene = auto_aim_visualizer.make_scene(
        targets, command.control ? bullet_speed : invalid,
        command.control ? command.yaw : invalid,
        command.control ? command.pitch : invalid, attitude, timestamp_ns);
      foxglove->publish(
        timestamp_ns,
        {{2, auto_aim_visualizer.make_gimbal_transform(attitude, timestamp_ns).dump()},
         {1, scene.dump()}});
    }
    /// 调试输出

    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    tools::draw_text(
      img,
      fmt::format(
        "command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,
        command.pitch * 57.3, command.shoot),
      {10, 60}, {154, 50, 205});

    Eigen::Quaternion gimbal_q = {w, x, y, z};
    tools::draw_text(
      img,
      fmt::format(
        "gimbal yaw{:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
      {10, 90}, {255, 255, 255});

    nlohmann::json data;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    Eigen::Quaternion q{w, x, y, z};
    auto yaw = tools::eulers(q, 2, 1, 0)[0];
    data["gimbal_yaw"] = yaw * 57.3;
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;

    if (!targets.empty()) {
      auto target = targets.front();

      if (last_t == -1) {
        last_target = target;
        last_t = t;
        continue;
      }

      std::vector<Eigen::Vector4d> armor_xyza_list;

      // 当前帧target更新后
      armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h1"] = x[10];
      data["h2"] = x[11];
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last_reprojection_time);
    last_reprojection_time = now;
    tools::draw_text(img, fmt::format("FPS: {:.1f}", 1.0 / dt), {10, 120}, {255, 255, 255});
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(30);
    if (key == 'q') break;
  }

  // Keep the last published snapshot and the WebSocket connection alive after a
  // replay reaches EOF. Otherwise Foxglove sees the server disappear and starts
  // its automatic reconnect loop, which looks like a recurring disconnect.
  if (input_exhausted && foxglove && !exiter.exit()) {
    tools::logger()->info("Replay input ended; keeping Foxglove connected (press q or Ctrl-C to exit)");
    while (!exiter.exit()) {
      if (cv::waitKey(50) == 'q') break;
    }
  }

  return 0;
}
