#ifndef IO__SUBSCRIBE2DECISION_HPP
#define IO__SUBSCRIBE2DECISION_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <sentry_interfaces/msg/decision_ack.hpp>
#include <sentry_interfaces/msg/decision_command.hpp>

#include "io/gimbal/gimbal.hpp"

namespace io
{
// 订阅决策下发的统一动作命令，待 MCU 决策下行帧确定后打包经串口下发，
// 并把执行回执以 DecisionAck 发布。契约见决策仓库 docs/INTERFACES.md。
class Subscribe2Decision : public rclcpp::Node
{
public:
  explicit Subscribe2Decision(Gimbal & gimbal);

  ~Subscribe2Decision() override;

  void start();

private:
  void command_callback(const sentry_interfaces::msg::DecisionCommand::SharedPtr msg);

  Gimbal & gimbal_;
  rclcpp::Subscription<sentry_interfaces::msg::DecisionCommand>::SharedPtr command_sub_;
  rclcpp::Publisher<sentry_interfaces::msg::DecisionAck>::SharedPtr ack_pub_;
};
}  // namespace io

#endif  // IO__SUBSCRIBE2DECISION_HPP
