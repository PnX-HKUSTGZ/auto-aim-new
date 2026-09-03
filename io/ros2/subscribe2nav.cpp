#include "subscribe2nav.hpp"

#include <functional>
#include <string>

namespace io
{
Subscribe2Nav::Subscribe2Nav(Gimbal & gimbal)
: Node("gimbal_navigation_bridge"), gimbal_(gimbal)
{
  navigation_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)),
    std::bind(&Subscribe2Nav::navigation_callback, this, std::placeholders::_1));

  change_follow_mark_service_ = create_service<std_srvs::srv::SetBool>(
    "/change_follow_mark",
    std::bind(
      &Subscribe2Nav::change_follow_mark_callback, this, std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Navigation-to-gimbal bridge initialized");
}

Subscribe2Nav::~Subscribe2Nav()
{
  RCLCPP_INFO(get_logger(), "Navigation-to-gimbal bridge shutting down");
}

void Subscribe2Nav::navigation_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  gimbal_.send_navigation(
    static_cast<float>(msg->linear.x), static_cast<float>(msg->linear.y),
    static_cast<float>(msg->linear.z), static_cast<float>(msg->angular.x),
    static_cast<float>(msg->angular.y), static_cast<float>(msg->angular.z));
}

void Subscribe2Nav::change_follow_mark_callback(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  const auto follow_mark = gimbal_.set_follow_mark(request->data);
  response->success = true;
  response->message = "follow_mark=" + std::to_string(follow_mark);
}

void Subscribe2Nav::start()
{
  rclcpp::spin(shared_from_this());
}
}  // namespace io
