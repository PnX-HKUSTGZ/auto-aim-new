#include "ros2.hpp"

namespace io
{
ROS2::ROS2()
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);

  publish2nav_ = std::make_shared<Publish2Nav>();
  publish_spin_thread_ = std::make_unique<std::thread>([this]() { publish2nav_->start(); });
}

ROS2::ROS2(Gimbal & gimbal) : ROS2()
{
  subscribe2nav_ = std::make_shared<Subscribe2Nav>(gimbal);
  publish2decision_making_ = std::make_shared<Publish2DecisionMaking>(gimbal);

  subscribe_spin_thread_ = std::make_unique<std::thread>([this]() { subscribe2nav_->start(); });
  decision_spin_thread_ =
    std::make_unique<std::thread>([this]() { publish2decision_making_->start(); });
}

ROS2::~ROS2()
{
  rclcpp::shutdown();

  if (publish_spin_thread_ && publish_spin_thread_->joinable()) publish_spin_thread_->join();
  if (subscribe_spin_thread_ && subscribe_spin_thread_->joinable()) subscribe_spin_thread_->join();
  if (decision_spin_thread_ && decision_spin_thread_->joinable()) decision_spin_thread_->join();
}

void ROS2::publish(const Eigen::Vector4d & target_pos)
{
  publish2nav_->send_data(target_pos);
}

std::vector<int8_t> ROS2::subscribe_enemy_status()
{
  return {};
}

std::vector<int8_t> ROS2::subscribe_autoaim_target()
{
  return {};
}
}  // namespace io
