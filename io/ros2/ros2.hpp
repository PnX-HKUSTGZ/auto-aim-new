#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP

#include <Eigen/Dense>

#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#ifdef SP_VISION_WITH_ROS2

#include "publish2DecisionMaking.hpp"
#include "publish2nav.hpp"
#include "subscribe2nav.hpp"

#endif

namespace io
{
class Gimbal;

class ROS2
{
public:
#ifdef SP_VISION_WITH_ROS2
  ROS2();

  explicit ROS2(Gimbal & gimbal);

  ~ROS2();

  void publish(const Eigen::Vector4d & target_pos);

  std::vector<int8_t> subscribe_enemy_status();

  std::vector<int8_t> subscribe_autoaim_target();

  template <typename T>
  std::shared_ptr<rclcpp::Publisher<T>> create_publisher(
    const std::string & node_name, const std::string & topic_name, size_t queue_size)
  {
    auto node = std::make_shared<rclcpp::Node>(node_name);
    auto publisher = node->create_publisher<T>(topic_name, queue_size);
    std::thread([node]() { rclcpp::spin(node); }).detach();
    return publisher;
  }

private:
  std::shared_ptr<Publish2Nav> publish2nav_;
  std::shared_ptr<Subscribe2Nav> subscribe2nav_;
  std::shared_ptr<Publish2DecisionMaking> publish2decision_making_;

  std::unique_ptr<std::thread> publish_spin_thread_;
  std::unique_ptr<std::thread> subscribe_spin_thread_;
  std::unique_ptr<std::thread> decision_spin_thread_;
#else
  ROS2() = default;

  explicit ROS2(Gimbal &) {}

  ~ROS2() = default;

  void publish(const Eigen::Vector4d &) {}

  std::vector<int8_t> subscribe_enemy_status() { return {}; }

  std::vector<int8_t> subscribe_autoaim_target() { return {}; }
#endif
};

}  // namespace io
#endif
