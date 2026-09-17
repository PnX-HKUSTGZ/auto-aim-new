#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <tuple>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth}
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
}

std::string Tracker::state() const { return state_; }

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
  bool use_enemy_color)  //被主函数调用，返回targets
{
  // 首帧时间可能来自离线回放；后续只接收严格递增时间戳，避免倒退预测。
  if (has_timestamp_ && t <= last_timestamp_) return {};
  has_timestamp_ = true;
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 参数为 false 时允许调用者自行决定颜色筛选策略。
  if (use_enemy_color) armors.remove_if([&](const Armor & a) { return a.color != enemy_color_; });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  // 每帧先将所有候选装甲板解算到世界系，供跟踪和可视化使用。
  solver_.solve(armors);
  // PnP 失败会把类别改为 not_armor，必须在读取队首优先级之前移除。
  armors.remove_if([](const Armor & armor) {
    return armor.name == ArmorName::not_armor || !armor.xyz_in_world.allFinite() ||
           !armor.ypd_in_world.allFinite() || !armor.ypr_in_world.allFinite();
  });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);  //初始化target
  }

  else {
    found = update_target(armors, t);  //更新target
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  // 四维/八维观测分别经过正确 NIS 检验，窗口足够长后才判断持续异常。
  if (state_ != "lost" && target_.bad_quality()) {
    state_ = "lost";
    return {};
  }

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};
  return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  for (const auto & detection : detection_queue) {
    temp_target = detection;
    temp_target.armors.remove_if([&](const Armor & armor) {
      return armor.name == ArmorName::not_armor || (use_enemy_color && armor.color != enemy_color_);
    });
    if (!temp_target.armors.empty()) break;
  }
  if (use_enemy_color)
    armors.remove_if([&](const Armor & armor) { return armor.color != enemy_color_; });

  // 首帧时间可能来自离线回放；后续只接收严格递增时间戳，避免倒退预测。
  if (has_timestamp_ && t <= last_timestamp_) return {switch_target, {}};
  has_timestamp_ = true;
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  // 每帧先将所有候选装甲板解算到世界系，供跟踪和可视化使用。
  solver_.solve(armors);
  // PnP 失败会把类别改为 not_armor，必须在读取队首优先级之前移除。
  armors.remove_if([](const Armor & armor) {
    return armor.name == ArmorName::not_armor || !armor.xyz_in_world.allFinite() ||
           !armor.ypd_in_world.allFinite() || !armor.ypr_in_world.allFinite();
  });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    // 新目标重新确认，不能继承旧目标 tracking 状态和检测次数。
    state_ = "lost";
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.convergened()) {
    state_ = "switching";
    temp_lost_count_ = 0;
    detect_count_ = 0;
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    const auto & expected = temp_target.armors.front();
    omni_target_priority_ = expected.priority;
    omni_target_name_ = expected.name;
    omni_target_type_ = expected.type;
    omni_target_color_ = expected.color;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    // 查找预期类别而非仅比较优先级；命中当帧就初始化，避免输出旧目标。
    std::list<Armor> expected_armors;
    for (const auto & armor : armors) {
      if (
        armor.name == omni_target_name_ && armor.type == omni_target_type_ &&
        armor.color == omni_target_color_ && armor.priority == omni_target_priority_)
        expected_armors.push_back(armor);
    }
    found = !expected_armors.empty() && set_target(expected_armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ != "lost" && state_ != "switching" && target_.bad_quality()) state_ = "lost";
  if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    detect_count_ = 1;
    temp_lost_count_ = 0;
    state_ = detect_count_ >= min_detect_count_ ? "tracking" : "detecting";
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      detect_count_ = 1;
      temp_lost_count_ = 0;
      state_ = detect_count_ >= min_detect_count_ ? "tracking" : "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
      temp_lost_count_ = 0;
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  for (auto & armor : armors) {
    if (armor.name == ArmorName::not_armor) continue;

    // 实际车型仅有普通四板和前哨站三板，与单帧可见板数无关。
    if (armor.name == ArmorName::outpost) {
      Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 1, 1}};
      target_ = Target(armor, t, 0.2765, 3, P0_dig);
    }

    else if (armor.name == ArmorName::base) {
      Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0, 0}};
      target_ = Target(armor, t, 0.3205, 4, P0_dig);
    }

    else {
      Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1, 0}};
      target_ = Target(armor, t, 0.2, 4, P0_dig);
    }

    return true;
  }

  return false;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);

  std::list<Armor> aim_armors;
  for (const auto & armor : armors) {
    if (armor.name == target_.name && armor.type == target_.armor_type) aim_armors.push_back(armor);
  }
  // 接受关联并完成融合才算 found；异常观测不推动 detecting/tracking 状态。
  if (aim_armors.empty()) return false;
  if (aim_armors.size() > 2)
    tools::logger()->warn("[Tracker] More than 2 candidates, selecting a valid pair or single armor");
  return target_.update_armors(aim_armors);
}

}  // namespace auto_aim
