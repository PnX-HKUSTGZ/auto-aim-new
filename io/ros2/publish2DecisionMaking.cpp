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

  referee_timer_ = create_wall_timer(10ms, [this]() { publish_referee_state(); });

  RCLCPP_INFO(get_logger(), "Referee uplink bridge initialized");
}

Publish2DecisionMaking::~Publish2DecisionMaking()
{
  RCLCPP_INFO(get_logger(), "Referee uplink bridge shutting down");
}

void Publish2DecisionMaking::publish_referee_state()
{
  const auto state = gimbal_.referee_state();
  if (state.sequence == 0 || state.sequence == last_referee_sequence_) return;
  const auto stamp = now();

  sentry_interfaces::msg::GameInfo game;
  game.header.stamp = stamp;
  // 统一帧带完整 game_status；旧 0x5A 只有 game_start，已在 Gimbal 内映射为 0/4。
  game.game_status = state.game_status;
  game.game_time_remaining = state.game_time_remaining;
  game.coin_remaining = state.coins;
  game.detect_color = state.detect_color;
  game.can_rebuild_outpost = state.can_rebuild_outpost;
  game.enemy_base_hp = state.enemy_base_hp;
  game.enemy_outpost_hp = state.enemy_outpost_hp;
  game_info_pub_->publish(game);

  sentry_interfaces::msg::SentryInfoOnline online;
  online.header.stamp = stamp;
  online.self_health = state.sentry_hp;
  online.bullets_remaining = state.remain_ammo;
  online.sentry_info_1 = state.sentry_info_1;
  online.sentry_info_2 = state.sentry_info_2;
  online_info_pub_->publish(online);

  sentry_interfaces::msg::TeamInfo team;
  team.header.stamp = stamp;
  team.base_hp = state.our_base_hp;
  team.outpost_hp = state.our_outpost_hp;
  team_info_pub_->publish(team);

  // SentryInfoOffline 的视觉锁定等字段待在 auto-aim 侧接入后发布。
  (void)offline_info_pub_;

  last_referee_sequence_ = state.sequence;
}

void Publish2DecisionMaking::start()
{
  rclcpp::spin(shared_from_this());
}
}  // namespace io
