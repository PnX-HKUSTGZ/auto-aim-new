#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>

#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct PlannerAimPoint
{
  bool valid;
  bool can_fire;  // 命中时刻是否处于射击窗口
  int id;         // 装甲板 id，-1 表示旋转中心跟踪
  Eigen::Vector4d xyza;
};

// 选板/锁板状态：轨迹采样内局部使用，仅在命令时刻提交
struct BoardSelectState
{
  int lock_id = -1;   // 当前锁定板
  std::chrono::steady_clock::time_point lock_since{}; // 实际或预测的换板时刻
};

struct Plan
{
  bool control;
  bool fire;
  float target_yaw;
  float target_yaw_vel;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza;
  Planner(const std::string & config_path);

  int lock_id() const { return lock_id_; }
  int aim_point_id() const { return debug_aim_point_id_; }

  Plan plan(Target target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed);

private:
  friend struct PlannerSelectionTest;
  double yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double fire_thresh_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;
  double air_resistance_ = 0.1;
  double low_speed_thresh_, high_speed_thresh_;  // 选板速度阈值（滞回），与 decision_speed_ 解耦
  double min_lock_hold_time_ = 0.0;              // 最短锁板保持时间（秒）
  double switch_margin_ = 0.0;                   // 新板须更优的角度余量（rad）
  bool high_speed_mode_ = false;                 // 当前选板速度模式（带滞回）
  int lock_id_ = -1;
  std::chrono::steady_clock::time_point lock_since_{};
  int debug_aim_point_id_ = -1;

  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;

  void setup_yaw_solver(const std::string & config_path);
  void setup_pitch_solver(const std::string & config_path);

  struct AimResult
  {
    Eigen::Matrix<double, 2, 1> yaw_pitch;
    PlannerAimPoint point;
  };

  AimResult aim(
    const Target & target, double bullet_speed, BoardSelectState & st,
    std::chrono::steady_clock::time_point time);
  PlannerAimPoint choose_aim_point(
    const Target & target, BoardSelectState & st, bool high_speed_mode,
    std::chrono::steady_clock::time_point time);
  Trajectory get_trajectory(
    Target target, double yaw0, double bullet_speed, const BoardSelectState & command_state,
    const AimResult & command, std::chrono::steady_clock::time_point time);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP