#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/auto_aim_scene.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "foxglove_scene_schema.hpp"
#include "tools/exiter.hpp"
#include "tools/foxglove_server.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }"
  "{foxglove       | true                   | 启用 Foxglove 三维可视化 }"
  "{foxglove-host  | 127.0.0.1              | Foxglove WebSocket 监听 IP }"
  "{foxglove-port  | 8765                   | Foxglove WebSocket 监听端口 }"
  "{frame-debug    | false                  | 每秒输出坐标系诊断数据 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  const bool foxglove_enabled = cli.get<bool>("foxglove");
  const bool frame_debug = cli.get<bool>("frame-debug");
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
           tools::kFoxgloveTransformSchema},
          {3, "/auto_aim/camera_transform", "foxglove.FrameTransform",
           tools::kFoxgloveTransformSchema}});
      tools::logger()->info(
        "Foxglove: ws://{}:{}, frame tree world -> gimbal -> camera",
        foxglove_host, foxglove_port);
    } catch (const std::exception & error) {
      tools::logger()->error("Cannot start Foxglove: {}", error.what());
      return 1;
    }
  }

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);
  auto_aim::AutoAimVisualizer auto_aim_visualizer(config_path);

  struct VisualizationInput
  {
    double bullet_speed = std::numeric_limits<double>::quiet_NaN();
    double yaw = std::numeric_limits<double>::quiet_NaN();
    double pitch = std::numeric_limits<double>::quiet_NaN();
  };
  std::mutex visualization_mutex;
  VisualizationInput latest_visualization_input;
  std::atomic<double> latest_gimbal_roll{std::numeric_limits<double>::quiet_NaN()};

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  auto pixel_plot_t0 = std::chrono::steady_clock::now();
  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      auto plan = planner.plan(target, gs.bullet_speed);
      auto now = std::chrono::steady_clock::now();

      VisualizationInput visualization_input;
      if (plan.control) {
        visualization_input = {gs.bullet_speed, plan.yaw, plan.pitch};
      }
      {
        std::lock_guard<std::mutex> lock(visualization_mutex);
        latest_visualization_input = visualization_input;
      }

      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;//可视化
      data["t"] = tools::delta_time(now, t0);

      // 电控回传角度为度, 显示统一转为弧度
      data["gimbal_yaw"] = tools::limit_rad(gs.yaw * M_PI / 180.0);
      data["gimbal_yaw_vel"] = gs.yaw_vel * M_PI / 180.0;
      data["gimbal_pitch"] = -gs.pitch * M_PI / 180.0;
      data["gimbal_pitch_vel"] = gs.pitch_vel * M_PI / 180.0;
      data["gimbal_roll"] = latest_gimbal_roll.load();

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;
      data["lock_id"] = planner.lock_id();
      data["aim_point_id"] = planner.aim_point_id();

      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
      }

      if (target.has_value()) {
        data["w"] = target->ekf_x()[7];
        data["a"] = target->ekf_x()[6];
        data["r"] = target->ekf_x()[8];
        data["target_x"] = target->ekf_x()[0];
        data["target_y"] = target->ekf_x()[2];
        data["target_vx"] = target->ekf_x()[1];
        data["target_vy"] = target->ekf_x()[3];
      } else {
        data["w"] = 0.0;
        data["r"] = 0.0;
        data["target_vx"] = 0.0;
        data["target_vy"] = 0.0;
      }

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  auto last_reprojection_time = std::chrono::steady_clock::now();
  auto last_frame_debug_time = last_reprojection_time;

  while (!exiter.exit()) {
    camera.read(img, t);
    auto q = gimbal.q(t - std::chrono::milliseconds(1));
    // auto q = Eigen::Quaterniond(1, 0, 0, 0);

    solver.set_R_gimbal2world(q);
    const auto gimbal_world_ypr = tools::eulers(q, 2, 1, 0);
    latest_gimbal_roll.store(gimbal_world_ypr[2]);
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    if (foxglove) {
      const auto timestamp_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());
      VisualizationInput visualization_input;
      {
        std::lock_guard<std::mutex> lock(visualization_mutex);
        visualization_input = latest_visualization_input;
      }
      const Eigen::Quaterniond attitude(solver.R_gimbal2world());
      foxglove->publish(
        timestamp_ns,
        {{2, auto_aim_visualizer.make_gimbal_transform(attitude, timestamp_ns).dump()},
         {3, auto_aim_visualizer.make_camera_transform(timestamp_ns).dump()},
         {1,
          auto_aim_visualizer
            .make_scene(
              targets, visualization_input.bullet_speed, visualization_input.yaw,
              visualization_input.pitch, attitude, timestamp_ns, armors)
            .dump()}});
    }

    nlohmann::json pixel_data;
    pixel_data["t"] = tools::delta_time(std::chrono::steady_clock::now(), pixel_plot_t0);
    pixel_data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      pixel_data["armor_center_pixel_x"] = armor.center.x;
      pixel_data["armor_center_pixel_y"] = armor.center.y;
      pixel_data["armor_center_norm_x"] = armor.center_norm.x;
      pixel_data["armor_center_norm_y"] = armor.center_norm.y;

      if (armor.xyz_in_world.allFinite()) {
        pixel_data["xyz_world_x"] = armor.xyz_in_world.x();
        pixel_data["xyz_world_y"] = armor.xyz_in_world.y();
        pixel_data["xyz_world_z"] = armor.xyz_in_world.z();
      }

      for (int i = 0; i < static_cast<int>(armor.points.size()); i++) {
        pixel_data[fmt::format("armor_{}_pixel_x", i)] = armor.points[i].x;
        pixel_data[fmt::format("armor_{}_pixel_y", i)] = armor.points[i].y;
      }
    }

    // 可视化未经 EKF 修正的基准装甲板角度 a。
    // Tracker 会依次更新所有同名、同类型装甲板，因此反向查找最后一块参与更新的装甲板，
    // 使其与 target.last_id 指向同一次观测。
    if (!targets.empty()) {
      const auto & target = targets.front();
      const auto armor_num = static_cast<int>(target.armor_xyza_list().size());
      for (auto armor_it = armors.rbegin(); armor_it != armors.rend(); ++armor_it) {
        if (armor_it->name != target.name || armor_it->type != target.armor_type) continue;

        const auto observed_a = armor_it->ypr_in_world[0];
        const int observed_id = target.last_id.empty() ? 0 : target.last_id.back();
        const auto raw_a = tools::limit_rad(
          observed_a - observed_id * 2.0 * CV_PI / static_cast<double>(armor_num));
        pixel_data["a_raw"] = raw_a;
        pixel_data["a_observed"] = observed_a;
        pixel_data["a_observed_id"] = observed_id;
        break;
      }
    }
    plotter.plot(pixel_data);
    const auto debug_now = std::chrono::steady_clock::now();
    if (frame_debug && tools::delta_time(debug_now, last_frame_debug_time) >= 1.0) {
      const auto world_ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      const auto feedback = gimbal.state();
      nlohmann::json frame_data = {
        {"imu_q_w", q.w()}, {"imu_q_x", q.x()},
        {"imu_q_y", q.y()}, {"imu_q_z", q.z()},
        {"imu_q_valid", q.coeffs().allFinite() && q.norm() > 1e-6},
        {"gimbal_yaw_world_rad", world_ypr[0]},
        {"gimbal_pitch_world_rad", world_ypr[1]},
        {"gimbal_roll_world_rad", world_ypr[2]},
        {"feedback_yaw_raw", feedback.yaw},
        {"feedback_pitch_raw", feedback.pitch},
        {"observed_valid", false}};
      for (const auto & armor : armors) {
        if (armor.name == auto_aim::ArmorName::not_armor) continue;
        frame_data["observed_valid"] =
          armor.xyz_in_gimbal.allFinite() && armor.xyz_in_world.allFinite();
        frame_data["observed_x_gimbal"] = armor.xyz_in_gimbal.x();
        frame_data["observed_y_gimbal"] = armor.xyz_in_gimbal.y();
        frame_data["observed_z_gimbal"] = armor.xyz_in_gimbal.z();
        frame_data["observed_x_world"] = armor.xyz_in_world.x();
        frame_data["observed_y_world"] = armor.xyz_in_world.y();
        frame_data["observed_z_world"] = armor.xyz_in_world.z();
        // 仅作诊断：比较正反旋转的高度稳定性，不参与跟踪或控制。
        const Eigen::Vector3d inverse_world =
          solver.R_gimbal2world().transpose() * armor.xyz_in_gimbal;
        frame_data["observed_z_inverse_rotation"] = inverse_world.z();
        frame_data["observed_bearing_gimbal_rad"] =
          std::atan2(armor.xyz_in_gimbal.y(), armor.xyz_in_gimbal.x());
        frame_data["observed_bearing_world_rad"] = armor.ypd_in_world[0];
        break;
      }
      frame_data["tracked_valid"] = !targets.empty();
      if (!targets.empty()) {
        const auto & tracked = targets.front();
        const auto state = tracked.ekf_x();
        frame_data["ekf_z"] = state[4];
        frame_data["ekf_vz"] = state[5];
        frame_data["ekf_h1"] = state[10];
        frame_data["ekf_h2"] = state[11];
        frame_data["matched_armor_id"] =
          tracked.last_id.empty() ? 0 : tracked.last_id.back();
      }
      tools::logger()->info("[WorldFrame] {}", frame_data.dump());
      last_frame_debug_time = debug_now;
    }

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
    }

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last_reprojection_time);
    last_reprojection_time = now;
    tools::draw_text(img, fmt::format("FPS: {:.1f}", 1.0 / dt), {10, 60}, {255, 255, 255});
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}
