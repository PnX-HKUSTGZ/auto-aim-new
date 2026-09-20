#ifndef AUTO_BUFF__RUNE_DECISION_STATE_MACHINE_HPP
#define AUTO_BUFF__RUNE_DECISION_STATE_MACHINE_HPP

#include <chrono>
#include <cstddef>
#include <deque>
#include <optional>
#include <string>

#include "rune_tracker.hpp"

namespace auto_buff
{
enum class RuneDecisionState { Lost, Tracking, Recovering };

struct RuneDecisionTarget
{
  std::optional<RuneTarget> target;
  RuneDecisionState state = RuneDecisionState::Lost;
  double recovery_ratio = 0.0;
};

// Port of RP-26Rune's target-continuity and fire-control state machine. Ballistic
// solving deliberately remains in Aimer because this project uses a different
// world coordinate convention and trajectory implementation.
class RuneDecisionStateMachine
{
public:
  explicit RuneDecisionStateMachine(const std::string & config_path);

  RuneDecisionTarget update(
    const std::optional<RuneTarget> & observation, RuneTimestamp now,
    bool retain_target_when_missing = true);

  // ballistic_solution_valid means the aimer has a usable and stable solution.
  bool allow_fire(bool ballistic_solution_valid, RuneTimestamp now);

private:
  struct ModeConfig
  {
    double max_life_time = 5.7;
    double target_switch_threshold = 0.30;
    double critical_timeout = 5.0;
    double fire_cooldown_time_init = 0.7;
    double fire_cooldown_time = 0.7;
    double fire_remaining_time_threshold = 0.04;
    std::size_t switch_confirm_count = 5;
  };

  enum class FireState { CoolingDown, Ready, Firing };

  const ModeConfig & mode_config(bool is_big_rune) const;
  void rebuild(const RuneTarget & target, RuneTimestamp now);
  void reset_fire_cooldown(RuneTimestamp now);
  bool confirm_big_rune_target_switch() const;
  RuneDecisionTarget current_target(RuneTimestamp now);

  ModeConfig small_config_;
  ModeConfig big_config_;
  double data_life_ = 0.2;
  double recover_time_ = 0.3;

  bool valid_ = false;
  RuneTarget target_;
  RuneTimestamp built_time_{};
  std::deque<RuneTarget> pending_targets_;

  FireState fire_state_ = FireState::Ready;
  RuneTimestamp cooldown_until_{};
  RuneTimestamp fire_started_at_{};
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__RUNE_DECISION_STATE_MACHINE_HPP
