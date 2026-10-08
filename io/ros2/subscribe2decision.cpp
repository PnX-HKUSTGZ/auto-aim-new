#include "subscribe2decision.hpp"

#include <chrono>
#include <functional>

namespace io
{
Subscribe2Decision::Subscribe2Decision(Gimbal & gimbal)
: Node("gimbal_decision_command_bridge"), gimbal_(gimbal)
{
  command_sub_ = create_subscription<sentry_interfaces::msg::DecisionCommand>(
    "/sentry/decision_command", 10,
    std::bind(&Subscribe2Decision::command_callback, this, std::placeholders::_1));
  ack_pub_ = create_publisher<sentry_interfaces::msg::DecisionAck>("/sentry/decision_ack", 10);
  ack_timer_ =
    create_wall_timer(std::chrono::milliseconds(10), [this]() { publish_acks(); });

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
  command.mode = msg->action.mode;
  command.interval_ms = msg->action.interval_ms;
  command.value = msg->action.value;

  if (!gimbal_.send_decision_command(command)) {
    RCLCPP_WARN(get_logger(), "decision command enqueue failed request_id=%u", msg->request_id);
    return;
  }
  RCLCPP_INFO(
    get_logger(), "decision command sent request_id=%u kind=%u mode=%u",
    msg->request_id, static_cast<unsigned>(msg->action.kind),
    static_cast<unsigned>(msg->action.mode));
}

void Subscribe2Decision::publish_acks()
{
  io::DecisionAck ack;
  while (gimbal_.pop_decision_ack(&ack)) {
    sentry_interfaces::msg::DecisionAck msg;
    msg.header.stamp = now();
    msg.request_id = ack.request_id;
    msg.accepted = ack.accepted != 0;
    msg.code = ack.code;
    ack_pub_->publish(msg);
  }
}

void Subscribe2Decision::start()
{
  rclcpp::spin(shared_from_this());
}
}  // namespace io
