#include <cmath>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "tasks/auto_buff/big_rune_motion_estimator.hpp"
#include "tasks/auto_buff/small_rune_motion_estimator.hpp"

namespace
{
constexpr double kSmallAngularVelocity = 1.0472;
constexpr double kBigLinearVelocity = 1.1775;
constexpr double kSectorPhase = 2.0 * M_PI / 5.0;

auto_buff::RuneTimestamp timestamp_at(auto_buff::RuneTimestamp start, double seconds)
{
  return start + std::chrono::duration_cast<auto_buff::RuneTimestamp::duration>(
                   std::chrono::duration<double>(seconds));
}

auto_buff::RuneCandidateTarget make_candidate(
  double phase, auto_buff::RuneTimestamp timestamp)
{
  auto_buff::RuneCandidateTarget candidate;
  candidate.rune_center = Eigen::Vector3d(4.0, 0.0, 0.0);
  candidate.rune_plane_world_normal = Eigen::Vector3d::UnitX();
  candidate.start_vector = -Eigen::Vector3d::UnitY();
  candidate.phase = auto_buff::normalize_rune_phase(phase);
  candidate.capture_timestamp = timestamp;
  const Eigen::Vector3d radial =
    candidate.start_vector * std::cos(candidate.phase) +
    candidate.rune_plane_world_normal.cross(candidate.start_vector) *
      std::sin(candidate.phase);
  candidate.armor_module_center = candidate.rune_center + 0.7 * radial;
  return candidate;
}

bool phase_near(double lhs, double rhs, double tolerance)
{
  return std::abs(auto_buff::rune_phase_difference(lhs, rhs)) <= tolerance;
}

bool test_small_target_switch(const std::string & config_path)
{
  auto_buff::SmallRuneMotionEstimator estimator(config_path);
  const auto start = auto_buff::RuneTimestamp{};
  std::optional<auto_buff::RuneTarget> result;
  for (int frame = 0; frame < 20; ++frame) {
    const double time = frame * 0.01;
    result = estimator.estimate(
      make_candidate(kSmallAngularVelocity * time, timestamp_at(start, time)));
  }
  if (!result) {
    std::cerr << "small rune did not initialize\n";
    return false;
  }

  const double switch_time = 0.20;
  result = estimator.estimate(make_candidate(
    kSmallAngularVelocity * switch_time + kSectorPhase,
    timestamp_at(start, switch_time)));
  if (result) {
    std::cerr << "small rune did not reject the target-switch frame\n";
    return false;
  }

  const double recovery_time = 0.21;
  const double recovery_phase = kSmallAngularVelocity * recovery_time + kSectorPhase;
  result = estimator.estimate(
    make_candidate(recovery_phase, timestamp_at(start, recovery_time)));
  if (!result || !phase_near(result->phase, recovery_phase, 0.05)) {
    std::cerr << "small rune did not recover around the switched target\n";
    return false;
  }
  return true;
}

bool test_small_filter_timestamp_advances(const std::string & config_path)
{
  auto_buff::SmallRuneMotionEstimator estimator(config_path);
  const auto start = auto_buff::RuneTimestamp{};
  std::optional<auto_buff::RuneTarget> result;

  // A constant-speed observation sequence should remain valid after initialization.
  // This catches a stale filter timestamp: posterior phase is updated each frame, so
  // using time since initialization again eventually creates a false phase jump.
  for (int frame = 0; frame < 200; ++frame) {
    const double time = frame * 0.01;
    const auto timestamp = timestamp_at(start, time);
    result = estimator.estimate(make_candidate(kSmallAngularVelocity * time, timestamp));
    if (frame >= 19) {
      if (!result) {
        std::cerr << "small rune reset with continuous timestamps at frame " << frame << '\n';
        return false;
      }
      if (result->capture_timestamp != timestamp) {
        std::cerr << "small rune output timestamp does not match its filtered state\n";
        return false;
      }
      if (!phase_near(result->phase, kSmallAngularVelocity * time, 0.03)) {
        std::cerr << "small rune phase drifted because filter time was integrated repeatedly\n";
        return false;
      }
    }
  }
  return true;
}

bool test_big_target_switch_and_jump_recovery(const std::string & config_path)
{
  auto_buff::BigRuneMotionEstimator estimator(config_path);
  const auto start = auto_buff::RuneTimestamp{};
  std::optional<auto_buff::RuneTarget> result;
  for (int frame = 0; frame < 100; ++frame) {
    const double time = frame * 0.02;
    result = estimator.estimate(
      {make_candidate(kBigLinearVelocity * time, timestamp_at(start, time))}, 1);
  }
  if (!result) {
    std::cerr << "big rune did not initialize\n";
    return false;
  }

  const double switch_time = 2.0;
  const double switched_phase = kBigLinearVelocity * switch_time + kSectorPhase;
  result = estimator.estimate(
    {make_candidate(switched_phase, timestamp_at(start, switch_time))}, 1);
  if (!result ||
      !phase_near(
        auto_buff::predict_phase(*result, result->capture_timestamp), switched_phase, 0.08)) {
    std::cerr << "big rune did not compensate a one-blade target switch\n";
    return false;
  }

  bool reset_observed = false;
  for (int frame = 1; frame <= 12; ++frame) {
    const double time = switch_time + frame * 0.08;
    const double branch_offset = frame % 2 == 0 ? kSectorPhase : 0.0;
    result = estimator.estimate(
      {make_candidate(
         kBigLinearVelocity * time + branch_offset, timestamp_at(start, time))},
      1);
    if (!result) {
      reset_observed = true;
      break;
    }
  }
  if (!reset_observed) {
    std::cerr << "big rune did not invalidate the model after frequent jumps\n";
    return false;
  }

  const double recovery_time = 3.04;
  result = estimator.estimate(
    {make_candidate(
       kBigLinearVelocity * recovery_time + kSectorPhase,
       timestamp_at(start, recovery_time))},
    1);
  if (!result) {
    std::cerr << "big rune did not rebuild from its retained observation window\n";
    return false;
  }
  return true;
}
// 覆盖非网格频率、频率边界、正反转、滑窗时间原点移动和少量观测离群点。
bool test_big_sinusoidal_prediction(const std::string & config_path)
{
  for (double omega : {1.884, 1.9373, 2.0}) {
    for (double direction : {-1.0, 1.0}) {
      auto_buff::BigRuneMotionEstimator estimator(config_path);
      const auto start = auto_buff::RuneTimestamp{};
      const auto phase_at = [&](double time) {
        return direction * (-0.43 * std::cos(omega * time + 0.6) +
                            kBigLinearVelocity * time + 0.3);
      };
      std::optional<auto_buff::RuneTarget> result;
      for (int frame = 0; frame <= 500; ++frame) {
        const double time = frame * 0.02;
        const double noise = 0.002 * std::sin(frame * 1.7) +
                             (frame % 53 == 0 ? 0.12 : 0.0);
        const double blade_offset = frame >= 250 ? kSectorPhase : 0.0;
        result = estimator.estimate(
          {make_candidate(phase_at(time) + blade_offset + noise, timestamp_at(start, time))}, 1);
        if (frame >= 400) {
          if (!result) {
            std::cerr << "big sinusoidal fit became invalid\n";
            return false;
          }
          const double predicted = auto_buff::predict_phase(*result, timestamp_at(start, time + 0.4));
          if (!phase_near(predicted, phase_at(time + 0.4) + blade_offset, 0.025)) {
            std::cerr << "big sinusoidal prediction error, omega=" << omega
                      << " direction=" << direction << " frame=" << frame << '\n';
            return false;
          }
        }
      }
      if (std::abs(result->big_rune_motion_model.speed_angular_frequency - omega) > 0.008) {
        std::cerr << "big rune frequency did not converge\n";
        return false;
      }
    }
  }
  return true;
}
}  // namespace

int main(int argc, char * argv[])
{
  const std::string config_path = argc > 1 ? argv[1] : "configs/standard3.yaml";
  if (!test_small_filter_timestamp_advances(config_path)) return 1;
  if (!test_small_target_switch(config_path)) return 1;
  if (!test_big_target_switch_and_jump_recovery(config_path)) return 1;
  if (!test_big_sinusoidal_prediction(config_path)) return 1;
  std::cout << "rune motion estimator tests passed\n";
  return 0;
}
