#ifndef AUTO_BUFF__AIMER_HPP
#define AUTO_BUFF__AIMER_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <optional>
#include <vector>

#include "../auto_aim/planner/planner.hpp"
#include "io/command.hpp"
#include "io/gimbal/gimbal.hpp"
#include "rune_tracker.hpp"

namespace auto_buff
{
class Aimer
{
public:
  Aimer(const std::string & config_path);

  io::Command aim(
    RuneTargetTracker & tracker, double bullet_speed, bool to_now = true);

  auto_aim::Plan mpc_aim(
    RuneTargetTracker & tracker, const io::GimbalState & gs, bool to_now = true);

  double angle = 0;  ///
  double t_gap = 0;  ///

private:
  double yaw_offset_;
  double pitch_offset_;
  double air_resistance_ = 0.1;

  double predict_time_;

  int mistake_count_ = 0;
  bool switch_fanblade_ = false;

  double last_yaw_ = 0;
  double last_pitch_ = 0;

  // for mpc
  bool first_in_aimer_ = true;


  bool get_send_angle(
    const RuneTarget & target, double prediction_offset_seconds, double bullet_speed,
    bool to_now, double & yaw, double & pitch) const;

  void apply_recovery(
    const RuneTarget & target, double recovery_ratio, double & yaw, double & pitch) const;
};
}  // namespace auto_buff
#endif  // AUTO_AIM__AIMER_HPP
