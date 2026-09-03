#include "publish2DecisionMaking.hpp"

#include <chrono>
#include <functional>

namespace io
{
using namespace std::chrono_literals;

Publish2DecisionMaking::Publish2DecisionMaking(Gimbal & gimbal)
: Node("gimbal_decision_bridge"), gimbal_(gimbal)
{
  sentry_health_pub_ = create_publisher<std_msgs::msg::UInt16>("/ifhealth", 10);
  our_base_health_pub_ = create_publisher<std_msgs::msg::UInt16>("/our_base_health", 10);
  enemy_base_health_pub_ = create_publisher<std_msgs::msg::UInt16>("/enemy_base_health", 10);
  our_outpost_health_pub_ = create_publisher<std_msgs::msg::UInt16>("/our_outpost_health", 10);
  enemy_outpost_health_pub_ = create_publisher<std_msgs::msg::UInt16>("/enemy_outpost_health", 10);
  can_rebuild_outpost_pub_ = create_publisher<std_msgs::msg::Bool>("/can_rebuild_outpost", 10);
  remain_ammo_pub_ = create_publisher<std_msgs::msg::UInt16>("/remain_ammo", 10);

  decision_service_ = create_service<std_srvs::srv::SetBool>(
    "/set_bool",
    std::bind(
      &Publish2DecisionMaking::set_decision_callback, this, std::placeholders::_1,
      std::placeholders::_2));
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

  std_msgs::msg::UInt16 sentry_health;
  std_msgs::msg::UInt16 our_base_health;
  std_msgs::msg::UInt16 enemy_base_health;
  std_msgs::msg::UInt16 our_outpost_health;
  std_msgs::msg::UInt16 enemy_outpost_health;
  std_msgs::msg::Bool can_rebuild_outpost;
  std_msgs::msg::UInt16 remain_ammo;

  sentry_health.data = state.sentry_hp;
  our_base_health.data = state.our_base_hp;
  enemy_base_health.data = state.enemy_base_hp;
  our_outpost_health.data = state.our_outpost_hp;
  enemy_outpost_health.data = state.enemy_outpost_hp;
  can_rebuild_outpost.data = state.can_rebuild_outpost;
  remain_ammo.data = state.remain_ammo;

  sentry_health_pub_->publish(sentry_health);
  our_base_health_pub_->publish(our_base_health);
  enemy_base_health_pub_->publish(enemy_base_health);
  our_outpost_health_pub_->publish(our_outpost_health);
  enemy_outpost_health_pub_->publish(enemy_outpost_health);
  can_rebuild_outpost_pub_->publish(can_rebuild_outpost);
  remain_ammo_pub_->publish(remain_ammo);
  last_referee_sequence_ = state.sequence;
}

void Publish2DecisionMaking::set_decision_callback(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  response->success = gimbal_.send_decision(request->data, 1s);
  response->message = response->success ? "decision packet sent" : "decision packet send failed";
}

void Publish2DecisionMaking::start()
{
  rclcpp::spin(shared_from_this());
}
}  // namespace io
