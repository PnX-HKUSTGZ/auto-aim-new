#include "buff_aimer.hpp"
#include "rune_decision_state_machine.hpp"

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_buff
{
Aimer::Aimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;      // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;  // degree to rad
  air_resistance_ = yaml["air_resistance"].as<double>();
  predict_time_ = yaml["predict_time"].as<double>();
}

io::Command Aimer::aim(
  RuneTargetTracker & tracker, double bullet_speed, bool to_now)
{
  io::Command command = {false, false, 0, 0};
  const RuneDecisionTarget & decision = tracker.decision();
  if (!decision.target) {
    switch_fanblade_ = true;
    tracker.allow_fire(false);
    return command;
  }

  // 如果子弹速度小于10，将其设为24
  if (bullet_speed < 10) bullet_speed = 24;

  double yaw = 0.0;
  double pitch = 0.0;

  const bool angle_solved =
    get_send_angle(*decision.target, predict_time_, bullet_speed, to_now, yaw, pitch);
  if (angle_solved) {
    if (decision.state == RuneDecisionState::Recovering)
      apply_recovery(*decision.target, decision.recovery_ratio, yaw, pitch);
    command.yaw = yaw;
    command.pitch = -pitch;  //世界坐标系下的pitch向上为负
    if (mistake_count_ > 3) {
      switch_fanblade_ = true;
      mistake_count_ = 0;
      command.control = true;
    } else if (std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3) {
      switch_fanblade_ = true;
      mistake_count_++;
      command.control = false;
    } else {
      switch_fanblade_ = false;
      mistake_count_ = 0;
      command.control = true;
    }
    last_yaw_ = yaw;
    last_pitch_ = pitch;
  } else {
    switch_fanblade_ = true;
    tracker.allow_fire(false);
    return command;
  }

  const bool may_fire =
    command.control && !switch_fanblade_ && decision.state == RuneDecisionState::Tracking;
  command.shoot = tracker.allow_fire(may_fire);

  return command;
}

auto_aim::Plan Aimer::mpc_aim(
  RuneTargetTracker & tracker, const io::GimbalState & gs, bool to_now)
{
  auto_aim::Plan plan = {false, false, 0, 0, 0, 0, 0, 0, 0, 0};
  const RuneDecisionTarget & decision = tracker.decision();
  if (!decision.target) {
    switch_fanblade_ = true;
    first_in_aimer_ = true;
    tracker.allow_fire(false);
    return plan;
  }

  double bullet_speed;
  // 如果子弹速度小于10，将其设为24
  if (gs.bullet_speed < 10)
    bullet_speed = 24;
  else
    bullet_speed = gs.bullet_speed;

  double yaw = 0.0;
  double pitch = 0.0;

  const bool angle_solved =
    get_send_angle(*decision.target, predict_time_, bullet_speed, to_now, yaw, pitch);
  if (angle_solved) {
    if (decision.state == RuneDecisionState::Recovering)
      apply_recovery(*decision.target, decision.recovery_ratio, yaw, pitch);
    plan.yaw = yaw;
    plan.pitch = -pitch;  //世界坐标系下的pitch向上为负
    if (mistake_count_ > 3) {
      switch_fanblade_ = true;
      mistake_count_ = 0;
      plan.control = true;
      first_in_aimer_ = true;
    } else if (std::abs(last_yaw_ - yaw) > 5 / 57.3 || std::abs(last_pitch_ - pitch) > 5 / 57.3) {
      switch_fanblade_ = true;
      mistake_count_++;
      plan.control = false;

      first_in_aimer_ = true;
    } else {
      switch_fanblade_ = false;
      mistake_count_ = 0;
      plan.control = true;
    }
    last_yaw_ = yaw;
    last_pitch_ = pitch;

    if (plan.control) {
      if (decision.state == RuneDecisionState::Recovering) {
        plan.yaw_vel = 0;
        plan.yaw_acc = 0;
        plan.pitch_vel = 0;
        plan.pitch_acc = 0;
        first_in_aimer_ = true;
      } else if (first_in_aimer_) {
        plan.yaw_vel = 0;
        plan.yaw_acc = 0;
        plan.pitch_vel = 0;
        plan.pitch_acc = 0;
        first_in_aimer_ = false;
      } else {
        const double dt = std::max(std::abs(predict_time_), 0.01);
        double yaw_before = 0.0;
        double pitch_before = 0.0;
        double yaw_after = 0.0;
        double pitch_after = 0.0;
        const bool before_ok = get_send_angle(
          *decision.target, predict_time_ - dt, bullet_speed, to_now, yaw_before,
          pitch_before);
        const bool after_ok = get_send_angle(
          *decision.target, predict_time_ + dt, bullet_speed, to_now, yaw_after,
          pitch_after);
        if (before_ok && after_ok) {
          plan.yaw_vel = tools::limit_rad(yaw_after - yaw_before) / (2.0 * dt);
          plan.yaw_acc =
            tools::limit_rad(yaw_after - 2.0 * yaw + yaw_before) / (dt * dt);

          const double pitch_command = -pitch;
          const double pitch_before_command = -pitch_before;
          const double pitch_after_command = -pitch_after;
          plan.pitch_vel =
            tools::limit_rad(pitch_after_command - pitch_before_command) / (2.0 * dt);
          plan.pitch_acc =
            (pitch_after_command - 2.0 * pitch_command + pitch_before_command) / (dt * dt);
        }
      }
    }
  } else {
    switch_fanblade_ = true;
    first_in_aimer_ = true;
    tracker.allow_fire(false);
    return plan;
  }

  const bool may_fire =
    plan.control && !switch_fanblade_ && decision.state == RuneDecisionState::Tracking;
  plan.fire = tracker.allow_fire(may_fire);

  return plan;
}

bool Aimer::get_send_angle(
  const RuneTarget & target, double prediction_offset_seconds, double bullet_speed,
  bool to_now, double & yaw, double & pitch) const
{
  constexpr int kMaxTrajectoryIterations = 5;
  constexpr double kPositionConvergenceThreshold = 0.01;

  const double observation_age = to_now // 是否用真实时间的时间差来预测目标位置
                                   ? std::max(
                                       tools::delta_time(
                                         std::chrono::steady_clock::now(),
                                         target.capture_timestamp),
                                       0.0)
                                   : 0.1;
  const auto duration = std::chrono::duration_cast<RuneTimestamp::duration>( // 总的预测时间 = 图片弹道解算处理时间差 + 预测偏移时间
    std::chrono::duration<double>(observation_age + prediction_offset_seconds));
  const RuneTimestamp first_prediction_time = target.capture_timestamp + duration;

  Eigen::Vector3d aim_in_world = predict_armor_position(target, first_prediction_time);
  double d = std::hypot(aim_in_world.x(), aim_in_world.y());

  // 以首次预测位置创建初始弹道。
  tools::Trajectory trajectory(bullet_speed, d, aim_in_world.z(), air_resistance_);
  if (trajectory.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable initial trajectory: {:.2f} {:.2f} {:.2f}", bullet_speed, d,
      aim_in_world.z());
    return false;
  }

  for (int iteration = 0; iteration < kMaxTrajectoryIterations; ++iteration) {
    // 使用当前弹道的飞行时间再次预测，并与当前瞄准位置比较。
    const auto flight_duration = std::chrono::duration_cast<RuneTimestamp::duration>(
      std::chrono::duration<double>(trajectory.fly_time));
    const Eigen::Vector3d predicted_position =
      predict_armor_position(target, first_prediction_time + flight_duration);
    const double position_error = (predicted_position - aim_in_world).norm();

    aim_in_world = predicted_position;
    d = std::hypot(aim_in_world.x(), aim_in_world.y());
    trajectory = tools::Trajectory(bullet_speed, d, aim_in_world.z(), air_resistance_);
    if (trajectory.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iteration {}: {:.2f} {:.2f} {:.2f}",
        iteration + 1, bullet_speed, d, aim_in_world.z());
      return false;
    }

    if (position_error < kPositionConvergenceThreshold) {
      yaw = std::atan2(aim_in_world.y(), aim_in_world.x()) + yaw_offset_;
      pitch = trajectory.pitch + pitch_offset_;
      return true;
    }
  }

  tools::logger()->debug("[Aimer] Trajectory did not converge within 5 iterations");
  return false;
}

void Aimer::apply_recovery(
  const RuneTarget & target, double recovery_ratio, double & yaw, double & pitch) const
{
  const double center_yaw =
    std::atan2(target.rune_center.y(), target.rune_center.x()) + yaw_offset_;
  const double center_pitch =
    std::atan2(
      target.rune_center.z(),
      std::hypot(target.rune_center.x(), target.rune_center.y())) +
    pitch_offset_;
  yaw += recovery_ratio * tools::limit_rad(center_yaw - yaw);
  pitch += recovery_ratio * (center_pitch - pitch);
}

}  // namespace auto_buff
