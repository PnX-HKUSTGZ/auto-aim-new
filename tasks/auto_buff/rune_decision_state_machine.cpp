#include "rune_decision_state_machine.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

namespace auto_buff
{
namespace
{
template <typename T>
T value_or(const YAML::Node & node, const char * key, T fallback)
{
  const YAML::Node value = node ? node[key] : YAML::Node{};
  return value && value.IsScalar() ? value.as<T>() : fallback;
}

RuneTimestamp add_seconds(RuneTimestamp timestamp, double seconds)
{
  return timestamp + std::chrono::duration_cast<RuneTimestamp::duration>(
                       std::chrono::duration<double>(std::max(seconds, 0.0)));
}
}  // namespace

RuneDecisionStateMachine::RuneDecisionStateMachine(const std::string & config_path)
{
  const YAML::Node yaml = YAML::LoadFile(config_path);
  const YAML::Node decision = yaml["buff_decision"];
  const double legacy_fire_gap = value_or<double>(yaml, "fire_gap_time", 0.7);

  data_life_ = value_or<double>(decision, "data_life", data_life_);
  recover_time_ = value_or<double>(decision, "recover_time", recover_time_);

  const auto load_mode = [legacy_fire_gap](
                           const YAML::Node & node, ModeConfig defaults,
                           double default_initial_cooldown) {
    defaults.max_life_time =
      value_or<double>(node, "max_life_time", defaults.max_life_time);
    defaults.target_switch_threshold = value_or<double>(
      node, "target_switch_threshold", defaults.target_switch_threshold);
    defaults.critical_timeout =
      value_or<double>(node, "critical_timeout", defaults.critical_timeout);
    defaults.fire_cooldown_time =
      value_or<double>(node, "fire_cooldown_time", legacy_fire_gap);
    defaults.fire_cooldown_time_init = value_or<double>(
      node, "fire_cooldown_time_init", default_initial_cooldown);
    defaults.fire_remaining_time_threshold = value_or<double>(
      node, "fire_remaining_time_threshold", defaults.fire_remaining_time_threshold);
    defaults.switch_confirm_count = std::max<std::size_t>(
      1, value_or<std::size_t>(
           node, "switch_confirm_count", defaults.switch_confirm_count));
    return defaults;
  };

  ModeConfig small_defaults;
  small_defaults.fire_cooldown_time = legacy_fire_gap;
  small_defaults.fire_cooldown_time_init = legacy_fire_gap;
  small_config_ = load_mode(
    decision ? decision["small"] : YAML::Node{}, small_defaults, legacy_fire_gap);

  ModeConfig big_defaults;
  big_defaults.max_life_time = 20.0;
  big_defaults.fire_cooldown_time = legacy_fire_gap;
  big_defaults.fire_cooldown_time_init = 0.4;
  big_config_ =
    load_mode(decision ? decision["big"] : YAML::Node{}, big_defaults, 0.4);
}

const RuneDecisionStateMachine::ModeConfig & RuneDecisionStateMachine::mode_config(
  bool is_big_rune) const
{
  return is_big_rune ? big_config_ : small_config_;
}

void RuneDecisionStateMachine::rebuild(const RuneTarget & target, RuneTimestamp now) // 重置寿命，清空等待target，清空开火冷却
{
  target_ = target;
  built_time_ = target.capture_timestamp;
  valid_ = true;
  pending_targets_.clear();
  reset_fire_cooldown(now);
}

void RuneDecisionStateMachine::reset_fire_cooldown(RuneTimestamp now)
{
  fire_state_ = FireState::CoolingDown;
  cooldown_until_ = add_seconds(
    now, mode_config(target_.is_big_rune).fire_cooldown_time_init);
}

bool RuneDecisionStateMachine::confirm_big_rune_target_switch() const
{
  const ModeConfig & config = big_config_;
  if (pending_targets_.size() < config.switch_confirm_count) return false;

  for (std::size_t i = 1; i < pending_targets_.size(); ++i) {
    const RuneTarget & previous = pending_targets_[i - 1];
    const RuneTarget & current = pending_targets_[i];
    const double expected_phase = predict_phase(previous, current.capture_timestamp);
    if (std::abs(rune_phase_difference(current.phase, expected_phase)) >
        config.target_switch_threshold)
      return false;
  }
  return true;
}

RuneDecisionTarget RuneDecisionStateMachine::current_target(RuneTimestamp now)
{
  if (!valid_) return {};

  const ModeConfig & config = mode_config(target_.is_big_rune);
  const double age = std::max(tools::delta_time(now, target_.capture_timestamp), 0.0);
  if (age > config.critical_timeout || age > data_life_ + recover_time_) {
    valid_ = false;
    pending_targets_.clear();
    fire_state_ = FireState::Ready;
    return {};
  }

  if (age > data_life_) {
    const double ratio = recover_time_ > 0.0 ? (age - data_life_) / recover_time_ : 1.0;
    return {target_, RuneDecisionState::Recovering, std::clamp(ratio, 0.0, 1.0)};
  }
  return {target_, RuneDecisionState::Tracking, 0.0};
}

RuneDecisionTarget RuneDecisionStateMachine::update(
  const std::optional<RuneTarget> & observation, RuneTimestamp now,
  bool retain_target_when_missing)
{
  if (!observation) {
    if (!retain_target_when_missing) {
      valid_ = false;
      pending_targets_.clear();
      fire_state_ = FireState::Ready;
      return {};
    }
    return current_target(now);
  }

  const RuneTarget & incoming = *observation;
  const ModeConfig & config = mode_config(incoming.is_big_rune);
  const double lifetime = valid_
                            ? tools::delta_time(incoming.capture_timestamp, built_time_)
                            : 0.0;
  if (!valid_ || target_.is_big_rune != incoming.is_big_rune || // 符类型切换 & 超出最大寿命
      lifetime > config.max_life_time) { 
    rebuild(incoming, now);
    return current_target(now);
  }

  if (!incoming.is_big_rune) { // 小符不需要确认切换，直接更新
    const double phase_jump =
      std::abs(rune_phase_difference(incoming.phase, target_.phase));
    if (phase_jump > config.target_switch_threshold) rebuild(incoming, now);
    else target_ = incoming;
    return current_target(now);
  }

  const double expected_phase = predict_phase(target_, incoming.capture_timestamp);
  const double phase_jump =
    std::abs(rune_phase_difference(incoming.phase, expected_phase));
  pending_targets_.push_back(incoming);
  while (pending_targets_.size() > config.switch_confirm_count) // 超过待确认数量，删除最早的
    pending_targets_.pop_front();

  if (phase_jump > config.target_switch_threshold) { // 大符发生跳跃，进入待确认状态
    if (confirm_big_rune_target_switch()) rebuild(pending_targets_.back(), now);
    return current_target(now);
  }

  target_ = incoming;
  return current_target(now);
}

bool RuneDecisionStateMachine::allow_fire(
  bool ballistic_solution_valid, RuneTimestamp now)
{
  if (!ballistic_solution_valid || !valid_) {
    if (fire_state_ == FireState::Firing) fire_state_ = FireState::Ready;
    if (fire_state_ == FireState::CoolingDown && now >= cooldown_until_)
      fire_state_ = FireState::Ready;
    return false;
  }

  const ModeConfig & config = mode_config(target_.is_big_rune);
  if (fire_state_ == FireState::CoolingDown) {
    if (now < cooldown_until_) return false; // 冷却期不开火
    fire_state_ = FireState::Ready;
  }

  if (fire_state_ == FireState::Ready) {
    fire_state_ = FireState::Firing;
    fire_started_at_ = now;
    return config.fire_remaining_time_threshold > 0.0;
  }

  if (tools::delta_time(now, fire_started_at_) >= // 开火后状态
      config.fire_remaining_time_threshold) {
    fire_state_ = FireState::CoolingDown;
    cooldown_until_ = add_seconds(now, config.fire_cooldown_time);
    return false;
  }
  return true;
}
}  // namespace auto_buff
