#include "trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tools
{
namespace
{
constexpr double kGravity = 9.8;
constexpr double kMinT = 1e-3;
constexpr double kMinDenom = 1e-6;
constexpr double kMaxExpArg = 50.0;
constexpr double kMaxFlightTime = 3.0;

double sanitizeTimeGuess(double guess)
{
  if (!std::isfinite(guess) || guess < kMinT) return 0.05;
  return std::clamp(guess, kMinT, kMaxFlightTime);
}

Trajectory solveWithoutAirResistance(double bullet_speed, double horizontal_distance, double height)
{
  Trajectory result;
  if (
    !std::isfinite(bullet_speed) || !std::isfinite(horizontal_distance) ||
    !std::isfinite(height) || bullet_speed <= kMinDenom) {
    return result;
  }

  const double d = std::max(horizontal_distance, kMinT);
  const double a = kGravity * d * d / (2.0 * bullet_speed * bullet_speed);
  const double b = -d;
  const double c = a + height;
  const double delta = b * b - 4.0 * a * c;

  if (!std::isfinite(delta) || delta < 0.0 || std::abs(a) < kMinDenom) return result;

  const double sqrt_delta = std::sqrt(delta);
  const double tan_pitch_1 = (-b + sqrt_delta) / (2.0 * a);
  const double tan_pitch_2 = (-b - sqrt_delta) / (2.0 * a);
  const double pitch_1 = std::atan(tan_pitch_1);
  const double pitch_2 = std::atan(tan_pitch_2);
  const double t_1 = d / (bullet_speed * std::cos(pitch_1));
  const double t_2 = d / (bullet_speed * std::cos(pitch_2));

  if (!std::isfinite(t_1) || !std::isfinite(t_2) || t_1 <= 0.0 || t_2 <= 0.0) return result;

  result.unsolvable = false;
  result.pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  result.fly_time = (t_1 < t_2) ? t_1 : t_2;
  return result;
}

double horizontalDistanceAtTime(
  double t, double pitch, double bullet_speed, double air_resistance)
{
  if (
    !std::isfinite(t) || !std::isfinite(pitch) || !std::isfinite(bullet_speed) ||
    !std::isfinite(air_resistance) || air_resistance <= kMinDenom ||
    bullet_speed <= kMinDenom) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double log_arg = air_resistance * std::cos(pitch) * bullet_speed * t + 1.0;
  if (!std::isfinite(log_arg) || log_arg <= kMinDenom) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return std::log(log_arg) / air_resistance;
}

double verticalDistanceAtTime(
  double t, double pitch, double bullet_speed, double air_resistance)
{
  if (
    !std::isfinite(t) || !std::isfinite(pitch) || !std::isfinite(bullet_speed) ||
    !std::isfinite(air_resistance) || air_resistance <= kMinDenom ||
    bullet_speed <= kMinDenom) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double vy = bullet_speed * std::sin(pitch);
  const double decay_arg = std::clamp(-air_resistance * t, -kMaxExpArg, kMaxExpArg);
  return (vy + kGravity / air_resistance) * (1.0 - std::exp(decay_arg)) /
           air_resistance -
         (kGravity * t) / air_resistance;
}

double optimizeTime(
  double initial_guess, double pitch, double bullet_speed, double air_resistance,
  const HorizontalDistanceFn & get_horizontal_distance)
{
  const double safe_initial_guess = sanitizeTimeGuess(initial_guess);
  if (
    !get_horizontal_distance || !std::isfinite(pitch) || !std::isfinite(air_resistance) ||
    air_resistance <= kMinDenom || !std::isfinite(bullet_speed) ||
    bullet_speed <= kMinDenom || std::cos(pitch) <= kMinDenom) {
    return safe_initial_guess;
  }

  auto residual = [&](double t) {
    const double target_distance = get_horizontal_distance(t);
    const double bullet_distance =
      horizontalDistanceAtTime(t, pitch, bullet_speed, air_resistance);
    if (!std::isfinite(target_distance) || !std::isfinite(bullet_distance)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return bullet_distance - target_distance;
  };

  double best_t = safe_initial_guess;
  double best_abs_residual = std::numeric_limits<double>::infinity();

  auto remember_best = [&](double t, double r) {
    if (std::isfinite(r) && std::abs(r) < best_abs_residual) {
      best_abs_residual = std::abs(r);
      best_t = t;
    }
  };

  double low = kMinT;
  double f_low = residual(low);
  remember_best(low, f_low);

  double high = std::max(safe_initial_guess, 0.05);
  double f_high = residual(high);
  remember_best(high, f_high);

  for (
    int i = 0;
    i < 16 && (!std::isfinite(f_low) || !std::isfinite(f_high) || f_low * f_high > 0.0);
    ++i) {
    high = std::min(high * 1.5 + 0.02, kMaxFlightTime);
    f_high = residual(high);
    remember_best(high, f_high);
    if (high >= kMaxFlightTime) break;
  }

  if (!std::isfinite(f_low) || !std::isfinite(f_high) || f_low * f_high > 0.0) {
    return sanitizeTimeGuess(best_t);
  }

  for (int i = 0; i < 32; ++i) {
    const double mid = 0.5 * (low + high);
    const double f_mid = residual(mid);
    remember_best(mid, f_mid);
    if (!std::isfinite(f_mid)) break;
    if (std::abs(f_mid) < 1e-4) return sanitizeTimeGuess(mid);

    if (f_low * f_mid <= 0.0) {
      high = mid;
      f_high = f_mid;
    } else {
      low = mid;
      f_low = f_mid;
    }
  }

  return sanitizeTimeGuess(best_t);
}

}  // namespace

Trajectory::Trajectory(const double v0, const double d, const double h)
: Trajectory(v0, d, h, kDefaultAirResistance)
{
}

Trajectory::Trajectory(
  const double v0, const double d, const double h, const double air_resistance)
{
  *this = fixTiteratPitch(v0, d, h, air_resistance);
}

Trajectory fixTiteratPitch(
  double bullet_speed, double horizontal_distance, double height, double air_resistance)
{
  if (air_resistance <= kMinDenom) {
    return solveWithoutAirResistance(bullet_speed, horizontal_distance, height);
  }

  Trajectory result;
  if (
    !std::isfinite(horizontal_distance) || !std::isfinite(height) ||
    !std::isfinite(air_resistance) || !std::isfinite(bullet_speed) ||
    bullet_speed <= kMinDenom) {
    return result;
  }

  const double dist_horizon = std::max(horizontal_distance, kMinT);
  const double target_height = height;
  double tmp_height = target_height;
  double tmp_pitch = std::atan2(target_height, dist_horizon);
  double fly_time = std::max(kMinT, dist_horizon / std::max(bullet_speed, kMinDenom));
  bool failed = false;

  for (int i = 0; i < 10; ++i) {
    tmp_pitch = std::atan2(tmp_height, dist_horizon);

    const double vx = bullet_speed * std::cos(tmp_pitch);
    const double vy = bullet_speed * std::sin(tmp_pitch);
    if (!std::isfinite(vx) || !std::isfinite(vy) || vx <= kMinDenom) {
      failed = true;
      break;
    }

    const double exp_arg = std::clamp(air_resistance * dist_horizon, -kMaxExpArg, kMaxExpArg);
    fly_time = (std::exp(exp_arg) - 1.0) / (air_resistance * vx);
    if (!std::isfinite(fly_time) || fly_time <= kMinT || fly_time > kMaxFlightTime) {
      failed = true;
      break;
    }

    const double term = vy + kGravity / air_resistance;
    const double decay_arg = std::clamp(-air_resistance * fly_time, -kMaxExpArg, kMaxExpArg);
    const double real_height =
      term * (1.0 - std::exp(decay_arg)) / air_resistance -
      (kGravity * fly_time) / air_resistance;
    if (!std::isfinite(real_height)) {
      failed = true;
      break;
    }

    tmp_height += target_height - real_height;
  }

  if (!std::isfinite(tmp_pitch)) tmp_pitch = std::atan2(target_height, dist_horizon);
  if (failed || !std::isfinite(tmp_pitch) || !std::isfinite(fly_time) || fly_time <= 0.0) {
    return result;
  }

  const double final_height =
    verticalDistanceAtTime(fly_time, tmp_pitch, bullet_speed, air_resistance);
  if (!std::isfinite(final_height) || std::abs(final_height - target_height) > 0.05) {
    return result;
  }

  result.unsolvable = false;
  result.pitch = tmp_pitch;
  result.fly_time = sanitizeTimeGuess(fly_time);
  return result;
}

std::pair<double, double> iteration(
  const double & thres, double & init_pitch, double & init_t,
  const TargetPositionFn & get_gun_target,
  const HorizontalDistanceFn & get_horizontal_distance, double bullet_speed,
  double & t_out, double air_resistance)
{
  double pitch = init_pitch;
  double t = sanitizeTimeGuess(init_t);

  if (!get_gun_target || !get_horizontal_distance) {
    t_out = t;
    return {std::isfinite(pitch) ? pitch : 0.0, 0.0};
  }

  if (!std::isfinite(pitch)) {
    const Eigen::Vector3d target0 = get_gun_target(0.0);
    const double dist0 = std::hypot(target0.x(), target0.y());
    pitch = std::atan2(target0.z(), std::max(dist0, kMinT));
  }

  if (!std::isfinite(pitch)) {
    t_out = t;
    return {0.0, 0.0};
  }

  for (int i = 0; i < 100; ++i) {
    t = optimizeTime(t, pitch, bullet_speed, air_resistance, get_horizontal_distance);

    const Eigen::Vector3d new_target = get_gun_target(t);
    if (!new_target.allFinite()) {
      t = sanitizeTimeGuess(t);
      break;
    }

    const double pred_dist = std::hypot(new_target.x(), new_target.y());
    const double pred_height = new_target.z();
    const Trajectory updated =
      fixTiteratPitch(bullet_speed, pred_dist, pred_height, air_resistance);
    if (updated.unsolvable || !std::isfinite(updated.pitch)) {
      t = sanitizeTimeGuess(t);
      break;
    }

    const double differ = pitch - updated.pitch;
    pitch = updated.pitch;
    t = sanitizeTimeGuess(updated.fly_time);

    if (std::abs(differ) < thres) break;
  }

  t_out = sanitizeTimeGuess(t);
  Eigen::Vector3d last_target = get_gun_target(t_out);
  if (!last_target.allFinite()) last_target = get_gun_target(0.0);
  if (!last_target.allFinite()) return {pitch, 0.0};

  return {pitch, std::atan2(last_target.y(), last_target.x())};
}

}  // namespace tools
