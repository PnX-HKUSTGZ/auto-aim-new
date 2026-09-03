#ifndef IO__PUBLISH2DECISIONMAKING_HPP
#define IO__PUBLISH2DECISIONMAKING_HPP

#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "io/gimbal/gimbal.hpp"

namespace io
{
class Publish2DecisionMaking : public rclcpp::Node
{
public:
  explicit Publish2DecisionMaking(Gimbal & gimbal);

  ~Publish2DecisionMaking() override;

  void start();

private:
  void publish_referee_state();

  void set_decision_callback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);

  Gimbal & gimbal_;
  uint64_t last_referee_sequence_{0};

  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr sentry_health_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr our_base_health_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr enemy_base_health_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr our_outpost_health_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr enemy_outpost_health_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr can_rebuild_outpost_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr remain_ammo_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr decision_service_;
  rclcpp::TimerBase::SharedPtr referee_timer_;
};
}  // namespace io

#endif  // IO__PUBLISH2DECISIONMAKING_HPP
