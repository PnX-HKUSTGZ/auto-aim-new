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
  ack_pub_ = create_publisher<sentry_interfaces::msg::DecisionAck>("/sentry/decision_ack", 10);

  RCLCPP_INFO(get_logger(), "Decision-command bridge initialized");
}

Subscribe2Decision::~Subscribe2Decision()
{
  RCLCPP_INFO(get_logger(), "Decision-command bridge shutting down");
}

void Subscribe2Decision::command_callback(
  const sentry_interfaces::msg::DecisionCommand::SharedPtr msg)
{
  // TODO(接口): MCU 决策下行帧确定后，在此把 action 打包为串口帧下发，
  // 并在收到执行回执后通过 ack_pub_ 发布 DecisionAck。当前仅记录，不发送。
  RCLCPP_INFO(
    get_logger(), "received decision command request_id=%u kind=%u mode=%u value=%d",
    msg->request_id, static_cast<unsigned>(msg->action.kind),
    static_cast<unsigned>(msg->action.mode), msg->action.value);
  (void)gimbal_;
  (void)ack_pub_;
}

void Subscribe2Decision::start()
{
  rclcpp::spin(shared_from_this());
}
}  // namespace io
