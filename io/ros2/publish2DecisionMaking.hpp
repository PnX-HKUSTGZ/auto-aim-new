#ifndef IO__PUBLISH2DECISIONMAKING_HPP
#define IO__PUBLISH2DECISIONMAKING_HPP

#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <sentry_interfaces/msg/game_info.hpp>
#include <sentry_interfaces/msg/radar_info.hpp>
#include <sentry_interfaces/msg/sentry_info_offline.hpp>
#include <sentry_interfaces/msg/sentry_info_online.hpp>
#include <sentry_interfaces/msg/team_info.hpp>

#include "io/gimbal/gimbal.hpp"

namespace io
{
// 把下位机上报的裁判状态，按 sentry_interfaces 的富消息发布给决策。
// 契约见决策仓库 docs/INTERFACES.md。
class Publish2DecisionMaking : public rclcpp::Node
{
public:
  explicit Publish2DecisionMaking(Gimbal & gimbal);

  ~Publish2DecisionMaking() override;

  void start();

private:
  void publish_referee_state();

  Gimbal & gimbal_;
  uint64_t last_referee_sequence_{0};

  rclcpp::Publisher<sentry_interfaces::msg::GameInfo>::SharedPtr game_info_pub_;
  rclcpp::Publisher<sentry_interfaces::msg::SentryInfoOnline>::SharedPtr online_info_pub_;
  rclcpp::Publisher<sentry_interfaces::msg::SentryInfoOffline>::SharedPtr offline_info_pub_;
  rclcpp::Publisher<sentry_interfaces::msg::TeamInfo>::SharedPtr team_info_pub_;
  rclcpp::Publisher<sentry_interfaces::msg::RadarInfo>::SharedPtr radar_info_pub_;
  rclcpp::TimerBase::SharedPtr referee_timer_;
};
}  // namespace io

#endif  // IO__PUBLISH2DECISIONMAKING_HPP
