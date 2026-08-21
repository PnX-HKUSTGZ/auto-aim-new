#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

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

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

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

      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;//可视化
      data["t"] = tools::delta_time(now, t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

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

  while (!exiter.exit()) {
    camera.read(img, t);
    auto q = gimbal.q(t - std::chrono::milliseconds(1));
    // auto q = Eigen::Quaterniond(1, 0, 0, 0);

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    nlohmann::json pixel_data;
    pixel_data["t"] = tools::delta_time(std::chrono::steady_clock::now(), pixel_plot_t0);
    pixel_data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      pixel_data["armor_center_pixel_x"] = armor.center.x;
      pixel_data["armor_center_pixel_y"] = armor.center.y;
      pixel_data["armor_center_norm_x"] = armor.center_norm.x;
      pixel_data["armor_center_norm_y"] = armor.center_norm.y;

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
        const auto raw_a = tools::limit_rad(
          observed_a - target.last_id * 2.0 * CV_PI / static_cast<double>(armor_num));
        pixel_data["a_raw"] = raw_a;
        pixel_data["a_observed"] = observed_a;
        pixel_data["a_observed_id"] = target.last_id;
        break;
      }
    }
    plotter.plot(pixel_data);

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