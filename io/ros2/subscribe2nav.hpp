#ifndef IO__SUBSCRIBE2NAV_HPP
#define IO__SUBSCRIBE2NAV_HPP

#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "io/gimbal/gimbal.hpp"

namespace io
{
class Subscribe2Nav : public rclcpp::Node
{
public:
  explicit Subscribe2Nav(Gimbal & gimbal);

  ~Subscribe2Nav() override;

  void start();

private:
  void navigation_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void change_follow_mark_callback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);

  Gimbal & gimbal_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigation_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr change_follow_mark_service_;
};
}  // namespace io

#endif  // IO__SUBSCRIBE2NAV_HPP
