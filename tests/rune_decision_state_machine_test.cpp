#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

#include "tasks/auto_buff/rune_decision_state_machine.hpp"

namespace
{
using namespace std::chrono_literals;

auto_buff::RuneTarget make_target(
  auto_buff::RuneTimestamp timestamp, double phase, bool is_big_rune)
{
  auto_buff::RuneTarget target;
  target.capture_timestamp = timestamp;
  target.phase = phase;
  target.is_big_rune = is_big_rune;
  target.big_rune_motion_model.reference_timestamp = timestamp;
  target.big_rune_motion_model.phase_constant_offset_radians = phase;
  return target;
}

void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string config_path = argc > 1 ? argv[1] : "configs/standard4.yaml";
  const auto t0 = std::chrono::steady_clock::now();

  auto_buff::RuneDecisionStateMachine fire_state(config_path);
  const auto small = make_target(t0, 0.0, false);
  require(
    fire_state.update(small, t0).state == auto_buff::RuneDecisionState::Tracking,
    "new target must enter tracking");
  require(!fire_state.allow_fire(true, t0), "initial cooldown must block fire");
  require(!fire_state.allow_fire(true, t0 + 699ms), "initial cooldown ended early");
  require(fire_state.allow_fire(true, t0 + 700ms), "initial cooldown did not end");
  require(fire_state.allow_fire(true, t0 + 730ms), "fire window ended early");
  require(!fire_state.allow_fire(true, t0 + 741ms), "fire window did not end");
  require(!fire_state.allow_fire(true, t0 + 1400ms), "repeat cooldown ended early");
  require(fire_state.allow_fire(true, t0 + 1442ms), "repeat cooldown did not end");

  auto_buff::RuneDecisionStateMachine timeout_state(config_path);
  timeout_state.update(small, t0);
  const auto recovering = timeout_state.update(std::nullopt, t0 + 250ms);
  require(
    recovering.state == auto_buff::RuneDecisionState::Recovering,
    "stale target must enter recovery");
  require(
    std::abs(recovering.recovery_ratio - 1.0 / 6.0) < 1e-6,
    "recovery interpolation ratio is wrong");
  require(
    timeout_state.update(std::nullopt, t0 + 501ms).state ==
      auto_buff::RuneDecisionState::Lost,
    "expired target must be lost");

  auto_buff::RuneDecisionStateMachine switch_state(config_path);
  switch_state.update(make_target(t0, 0.0, true), t0);
  for (int i = 1; i <= 4; ++i) {
    const auto timestamp = t0 + i * 20ms;
    const auto decision = switch_state.update(make_target(timestamp, 1.0, true), timestamp);
    require(decision.target.has_value(), "pending switch lost the old target");
    require(std::abs(decision.target->phase) < 1e-9, "big-rune switch confirmed early");
  }
  const auto switch_time = t0 + 100ms;
  const auto switched = switch_state.update(make_target(switch_time, 1.0, true), switch_time);
  require(switched.target.has_value(), "confirmed switch has no target");
  require(
    std::abs(switched.target->phase - 1.0) < 1e-9,
    "big-rune switch was not confirmed");
  require(
    !switch_state.allow_fire(true, switch_time),
    "confirmed switch must restart initial cooldown");

  std::cout << "rune decision state machine tests passed\n";
  return 0;
}
