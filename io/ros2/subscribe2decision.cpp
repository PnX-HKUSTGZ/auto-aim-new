#include "subscribe2decision.hpp"

#include <functional>

namespace io
{
Subscribe2Decision::Subscribe2Decision(Gimbal & gimbal)
: Node("gimbal_decision_command_bridge"), gimbal_(gimbal)
{
  command_sub_ = create_subscription<sentry_interfaces::msg::DecisionCommand>(
    "/sentry/decision_command", 10,
    std::bind(&Subscribe2Decision::command_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Decision-command bridge initialized");
}

Subscribe2Decision::~Subscribe2Decision()
{
  RCLCPP_INFO(get_logger(), "Decision-command bridge shutting down");
}

void Subscribe2Decision::command_callback(
  const sentry_interfaces::msg::DecisionCommand::SharedPtr msg)
{
  io::DecisionCommand command;
  command.request_id = msg->request_id;
  command.kind = msg->action.kind;
  command.value = msg->action.value;

  if (!gimbal_.send_decision_command(command)) {
    RCLCPP_WARN(get_logger(), "decision command enqueue failed request_id=%u", msg->request_id);
    return;
  }
  RCLCPP_INFO(
    get_logger(), "decision command sent request_id=%u kind=%u value=%d",
    msg->request_id, static_cast<unsigned>(msg->action.kind), msg->action.value);
}

void Subscribe2Decision::start()
{
  rclcpp::spin(shared_from_this());
}
}  // namespace io
