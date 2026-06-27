#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

#include <Eigen/Dense>

#include <functional>
#include <utility>

namespace tools
{
inline constexpr double kDefaultAirResistance = 0.1;

struct Trajectory
{
  bool unsolvable = true;
  double fly_time = 0.0;
  double pitch = 0.0;  // 抬头为正

  Trajectory() = default;

  // 考虑默认空气阻力
  // v0 子弹初速度大小，单位：m/s
  // d 目标水平距离，单位：m
  // h 目标竖直高度，单位：m
  Trajectory(const double v0, const double d, const double h);

  Trajectory(
    const double v0, const double d, const double h,
    const double air_resistance);
};

using TargetPositionFn = std::function<Eigen::Vector3d(double)>;
using HorizontalDistanceFn = std::function<double(double)>;

Trajectory fixTiteratPitch(
  double bullet_speed, double horizontal_distance, double height,
  double air_resistance = kDefaultAirResistance);

std::pair<double, double> iteration(
  const double & thres, double & init_pitch, double & init_t,
  const TargetPositionFn & get_gun_target,
  const HorizontalDistanceFn & get_horizontal_distance, double bullet_speed,
  double & t_out, double air_resistance = kDefaultAirResistance);

template <typename T>
std::pair<double, double> iteration(
  const double & thres, double & init_pitch, double & init_t, T & target_info,
  double bullet_speed, double & t_out, double air_resistance = kDefaultAirResistance)
{
  return iteration(
    thres, init_pitch, init_t,
    [&target_info](double t) { return target_info.getGunTarget(t); },
    [&target_info](double t) { return target_info.getHorizontalDistance(t); },
    bullet_speed, t_out, air_resistance);
}

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP
