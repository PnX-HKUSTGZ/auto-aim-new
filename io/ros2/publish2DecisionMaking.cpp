#include "publish2DecisionMaking.hpp"

#include <chrono>

namespace io
{
using namespace std::chrono_literals;

Publish2DecisionMaking::Publish2DecisionMaking(Gimbal & gimbal)
: Node("gimbal_decision_bridge"), gimbal_(gimbal)
{
  game_info_pub_ = create_publisher<sentry_interfaces::msg::GameInfo>("/sentry/game_info", 10);
  online_info_pub_ =
    create_publisher<sentry_interfaces::msg::SentryInfoOnline>("/sentry/online_info", 10);
  offline_info_pub_ =
    create_publisher<sentry_interfaces::msg::SentryInfoOffline>("/sentry/offline_info", 10);
  team_info_pub_ = create_publisher<sentry_interfaces::msg::TeamInfo>("/sentry/team_info", 10);
  radar_info_pub_ = create_publisher<sentry_interfaces::msg::RadarInfo>("/sentry/radar_info", 10);

  referee_timer_ = create_wall_timer(10ms, [this]() { publish_referee_state(); });

  RCLCPP_INFO(get_logger(), "Decision-to-gimbal bridge initialized");
}

Publish2DecisionMaking::~Publish2DecisionMaking()
{
  RCLCPP_INFO(get_logger(), "Decision-to-gimbal bridge shutting down");
}

void Publish2DecisionMaking::publish_referee_state()
{
  const auto state = gimbal_.referee_state();
  if (state.sequence == 0 || state.sequence == last_referee_sequence_) return;
  const auto stamp = now();

  sentry_interfaces::msg::GameInfo game;
  game.header.stamp = stamp;
  game.game_status = state.game_start ? 4 : 0;  // 阶段只有 0/4，完整语义待 MCU 扩展
  game.detect_color = state.detect_color;
  game.can_rebuild_outpost = state.can_rebuild_outpost;
  game.enemy_base_hp = state.enemy_base_hp;
  game.enemy_outpost_hp = state.enemy_outpost_hp;
  game_info_pub_->publish(game);

  sentry_interfaces::msg::SentryInfoOnline online;
  online.header.stamp = stamp;
  online.self_health = state.sentry_hp;
  online.bullets_remaining = state.remain_ammo;
  online_info_pub_->publish(online);

  sentry_interfaces::msg::TeamInfo team;
  team.header.stamp = stamp;
  team.base_hp = state.our_base_hp;
  team.outpost_hp = state.our_outpost_hp;
  team_info_pub_->publish(team);

  // SentryInfoOffline / RadarInfo 的字段 MCU 尚未提供，待上行帧扩展后再发布。
  (void)offline_info_pub_;
  (void)radar_info_pub_;

  last_referee_sequence_ = state.sequence;
}

void Publish2DecisionMaking::start()
{
  rclcpp::spin(shared_from_this());
}
}  // namespace io
