#ifndef IO__SUBSCRIBE2DECISION_HPP
#define IO__SUBSCRIBE2DECISION_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <sentry_interfaces/msg/decision_ack.hpp>
#include <sentry_interfaces/msg/decision_command.hpp>

#include "io/gimbal/gimbal.hpp"

namespace io
{
// 订阅决策下发的统一动作命令，按协议 v1 组帧经串口下发给 MCU，
// 并把 MCU 回执以 DecisionAck 发布。契约见 docs/serial_protocol.md §2 与决策仓库 docs/INTERFACES.md。
class Subscribe2Decision : public rclcpp::Node
{
public:
  explicit Subscribe2Decision(Gimbal & gimbal);

  ~Subscribe2Decision() override;

  void start();

private:
  void command_callback(const sentry_interfaces::msg::DecisionCommand::SharedPtr msg);

  // 把 Gimbal 收到的 MCU 回执转成 DecisionAck 发布。
  void publish_acks();

  Gimbal & gimbal_;
  rclcpp::Subscription<sentry_interfaces::msg::DecisionCommand>::SharedPtr command_sub_;
  rclcpp::Publisher<sentry_interfaces::msg::DecisionAck>::SharedPtr ack_pub_;
  rclcpp::TimerBase::SharedPtr ack_timer_;
};
}  // namespace io

#endif  // IO__SUBSCRIBE2DECISION_HPP
