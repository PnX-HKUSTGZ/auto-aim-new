#ifndef IO__SUBSCRIBE2DECISION_HPP
#define IO__SUBSCRIBE2DECISION_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <sentry_interfaces/msg/decision_command.hpp>

#include "io/gimbal/gimbal.hpp"

namespace io
{
// 订阅决策下发的统一动作命令，按协议 v1 组帧经串口下发给 MCU。
// MCU 只转达 0x0120 指令、不回执；决策侧用裁判上行校验生效并决定是否重发。
// 契约见 docs/serial_protocol.md §2 与决策仓库 docs/INTERFACES.md。
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
};
}  // namespace io

#endif  // IO__SUBSCRIBE2DECISION_HPP
