#include "gimbal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
template <typename T>
T read_or(const YAML::Node & yaml, const std::string & key, const T & default_value)
{
  return yaml[key] ? yaml[key].as<T>() : default_value;
}

std::chrono::steady_clock::duration frequency_to_period(double hz, double fallback_hz)
{
  if (!std::isfinite(hz) || hz <= 0.0) hz = fallback_hz;
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / hz));
}

template <typename Packet>
bool write_serial_packet(serial::Serial & serial, Packet packet)
{
  packet.crc16 = tools::get_crc16(
    reinterpret_cast<const uint8_t *>(&packet), sizeof(packet) - sizeof(packet.crc16));
  return serial.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet)) == sizeof(packet);
}
}  // namespace
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  const auto baud_rate = read_or<uint32_t>(yaml, "baud_rate", 115200);
  vision_min_period_ =
    frequency_to_period(read_or<double>(yaml, "vision_tx_max_hz", 100.0), 100.0);
  nav_min_period_ = frequency_to_period(read_or<double>(yaml, "nav_tx_max_hz", 50.0), 50.0);
  vision_timeout_ =
    std::chrono::milliseconds(std::max(0, read_or<int>(yaml, "vision_timeout_ms", 100)));
  nav_timeout_ =
    std::chrono::milliseconds(std::max(0, read_or<int>(yaml, "nav_timeout_ms", 200)));
  decision_queue_capacity_ =
    static_cast<std::size_t>(std::max(1, read_or<int>(yaml, "decision_queue_capacity", 16)));

  cmd_vel_linear_scale_ = read_or<double>(yaml, "cmd_vel_linear_scale", 0.4);
  follow_mark_default_value_ = static_cast<uint8_t>(std::clamp(
    read_or<int>(yaml, "follow_mark_default_value", 2), 0,
    static_cast<int>(std::numeric_limits<uint8_t>::max())));
  follow_mark_start_value_ = static_cast<uint8_t>(std::clamp(
    read_or<int>(yaml, "follow_mark_start_value", 1), 0,
    static_cast<int>(std::numeric_limits<uint8_t>::max())));
  follow_mark_rough_value_ = static_cast<uint8_t>(std::clamp(
    read_or<int>(yaml, "follow_mark_rough_value", 0), 0,
    static_cast<int>(std::numeric_limits<uint8_t>::max())));
  follow_mark_hold_nav_count_ =
    std::max(0, read_or<int>(yaml, "follow_mark_hold_nav_count", 10));
  follow_mark_zero_linear_scale_ =
    std::max(0.0, read_or<double>(yaml, "follow_mark_zero_linear_scale", 0.5));
  follow_mark_ = follow_mark_default_value_;

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(baud_rate);
    serial_.open();
    serial::Timeout timeout(10, 50, 0, 50, 0);
    serial_.setTimeout(timeout);
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  read_thread_ = std::thread(&Gimbal::read_thread, this);
  tx_thread_ = std::thread(&Gimbal::tx_loop, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  tx_cv_.notify_all();
  if (tx_thread_.joinable()) tx_thread_.join();
  if (read_thread_.joinable()) read_thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

RefereeState Gimbal::referee_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return referee_state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(const VisionToGimbal & command)
{
  {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    latest_vision_ = command;
    latest_vision_.header = 0xA5;
    vision_pending_ = true;
    has_vision_ = true;
    vision_watchdog_sent_ = false;
    vision_updated_at_ = Clock::now();
  }
  tx_cv_.notify_one();
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  VisionToGimbal command{};
  command.mode = control ? (fire ? 2 : 1) : 0;
  command.yaw = yaw;
  command.yaw_vel = yaw_vel;
  command.yaw_acc = yaw_acc;
  command.pitch = -pitch;
  command.pitch_vel = -pitch_vel;
  command.pitch_acc = -pitch_acc;
  send(command);
}

void Gimbal::send_navigation(
  float linear_x, float linear_y, float linear_z, float angular_x, float angular_y,
  float angular_z)
{
  {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    latest_nav_.linear_x = linear_x;
    latest_nav_.linear_y = linear_y;
    latest_nav_.linear_z = linear_z;
    latest_nav_.angular_x = angular_x;
    latest_nav_.angular_y = angular_y;
    latest_nav_.angular_z = angular_z;
    nav_pending_ = true;
    has_nav_ = true;
    nav_watchdog_sent_ = false;
    nav_updated_at_ = Clock::now();
  }
  tx_cv_.notify_one();
}

uint8_t Gimbal::set_follow_mark(bool enabled)
{
  std::lock_guard<std::mutex> lock(tx_mutex_);
  follow_mark_hold_remaining_ = 0;
  follow_mark_ = follow_mark_default_value_;
  if (enabled) {
    if (follow_mark_hold_nav_count_ > 0) {
      follow_mark_ = follow_mark_start_value_;
      follow_mark_hold_remaining_ = follow_mark_hold_nav_count_;
    } else {
      follow_mark_ = follow_mark_rough_value_;
    }
  }
  return follow_mark_;
}

bool Gimbal::send_decision(bool reload, std::chrono::milliseconds timeout)
{
  auto request = std::make_shared<DecisionRequest>();
  request->packet.ifreload = reload ? 1 : 0;
  auto result = request->completion.get_future();

  {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (quit_ || decision_queue_.size() >= decision_queue_capacity_) return false;
    decision_queue_.push_back(request);
  }
  tx_cv_.notify_one();

  if (result.wait_for(timeout) != std::future_status::ready) {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    const auto it = std::find(decision_queue_.begin(), decision_queue_.end(), request);
    if (it != decision_queue_.end()) {
      decision_queue_.erase(it);
      return false;
    }
  }

  try {
    return result.get();
  } catch (const std::future_error &) {
    return false;
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (reconnect_requested_.exchange(false)) {
      reconnect();
      continue;
    }

    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.header))) {
      error_count++;
      tools::logger()->debug("[Gimbal] read header failed, error_count={}", error_count);
      continue;
    }

    if (rx_data_.header != 0x5A) continue;

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.header),
          sizeof(rx_data_) - sizeof(rx_data_.header))) {
      error_count++;
      tools::logger()->debug("[Gimbal] read payload failed, error_count={}", error_count);
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    //Eigen::Quaterniond q(1, 0, 0, 0);
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(state_mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;

    referee_state_.sequence++;
    referee_state_.detect_color = rx_data_.detect_color;
    referee_state_.reset_tracker = rx_data_.reset_tracker;
    referee_state_.game_start = rx_data_.game_start;
    referee_state_.can_rebuild_outpost = rx_data_.can_rebuild_outpost;
    referee_state_.sentry_hp = rx_data_.sentryHP;
    referee_state_.our_base_hp = rx_data_.our_baseHP;
    referee_state_.enemy_base_hp = rx_data_.enemy_baseHP;
    referee_state_.our_outpost_hp = rx_data_.our_outpostHP;
    referee_state_.enemy_outpost_hp = rx_data_.enemy_outpostHP;
    referee_state_.remain_ammo = rx_data_.remain_ammo;

    switch (uint8_t(rx_data_.mode)) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", uint8_t(rx_data_.mode));
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::tx_loop()
{
  tools::logger()->info("[Gimbal] tx_thread started.");

  while (!quit_) {
    std::optional<VisionToGimbal> vision_packet;
    std::optional<NavToGimbalV2> nav_packet;
    std::shared_ptr<DecisionRequest> decision_request;

    {
      std::unique_lock<std::mutex> lock(tx_mutex_);
      while (!quit_) {
        const auto now = Clock::now();
        const bool decision_due = !decision_queue_.empty();
        const bool vision_due = vision_pending_ && now >= vision_next_send_at_;
        const bool nav_due = nav_pending_ && now >= nav_next_send_at_;
        const bool vision_watchdog_due =
          has_vision_ && !vision_watchdog_sent_ && vision_timeout_ > Clock::duration::zero() &&
          now >= vision_updated_at_ + vision_timeout_;
        const bool nav_watchdog_due =
          has_nav_ && !nav_watchdog_sent_ && nav_timeout_ > Clock::duration::zero() &&
          now >= nav_updated_at_ + nav_timeout_;

        if (decision_due || vision_due || nav_due || vision_watchdog_due || nav_watchdog_due) {
          if (decision_due) {
            decision_request = decision_queue_.front();
            decision_queue_.pop_front();
          }

          if (vision_due) {
            vision_packet = latest_vision_;
            vision_pending_ = false;
            vision_next_send_at_ = now + vision_min_period_;
          } else if (vision_watchdog_due) {
            VisionToGimbal stop{};
            stop.mode = 0;
            vision_packet = stop;
            vision_watchdog_sent_ = true;
            vision_next_send_at_ = now + vision_min_period_;
          }

          if (nav_due) {
            nav_packet = latest_nav_;
            nav_packet->follow_mark = follow_mark_;
            const double follow_scale =
              follow_mark_ == 0 ? follow_mark_zero_linear_scale_ : 1.0;
            nav_packet->linear_x = static_cast<float>(
              nav_packet->linear_x * cmd_vel_linear_scale_ * follow_scale);
            nav_packet->linear_y = static_cast<float>(
              nav_packet->linear_y * cmd_vel_linear_scale_ * follow_scale);
            nav_pending_ = false;
            nav_next_send_at_ = now + nav_min_period_;

            if (follow_mark_hold_remaining_ > 0) {
              --follow_mark_hold_remaining_;
              if (follow_mark_hold_remaining_ == 0) follow_mark_ = follow_mark_rough_value_;
            }
          } else if (nav_watchdog_due) {
            NavToGimbalV2 stop{};
            stop.follow_mark = follow_mark_;
            nav_packet = stop;
            nav_watchdog_sent_ = true;
            nav_next_send_at_ = now + nav_min_period_;
          }
          break;
        }

        auto wake_at = Clock::time_point::max();
        if (vision_pending_) wake_at = std::min(wake_at, vision_next_send_at_);
        if (nav_pending_) wake_at = std::min(wake_at, nav_next_send_at_);
        if (has_vision_ && !vision_watchdog_sent_ && vision_timeout_ > Clock::duration::zero()) {
          wake_at = std::min(wake_at, vision_updated_at_ + vision_timeout_);
        }
        if (has_nav_ && !nav_watchdog_sent_ && nav_timeout_ > Clock::duration::zero()) {
          wake_at = std::min(wake_at, nav_updated_at_ + nav_timeout_);
        }

        if (wake_at == Clock::time_point::max())
          tx_cv_.wait(lock);
        else
          tx_cv_.wait_until(lock, wake_at);
      }
    }

    if (quit_) {
      if (decision_request) decision_request->completion.set_value(false);
      break;
    }

    if (decision_request) {
      decision_request->completion.set_value(write_packet(decision_request->packet));
    }
    if (vision_packet) write_packet(*vision_packet);
    if (nav_packet) write_packet(*nav_packet);
  }

  std::lock_guard<std::mutex> lock(tx_mutex_);
  for (const auto & request : decision_queue_) request->completion.set_value(false);
  decision_queue_.clear();
  tools::logger()->info("[Gimbal] tx_thread stopped.");
}

bool Gimbal::write_packet(VisionToGimbal packet)
{
  try {
    if (write_serial_packet(serial_, packet)) return true;
    tools::logger()->warn("[Gimbal] Incomplete VisionToGimbal write.");
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write VisionToGimbal: {}", e.what());
  }
  reconnect_requested_ = true;
  return false;
}

bool Gimbal::write_packet(NavToGimbalV2 packet)
{
  try {
    if (write_serial_packet(serial_, packet)) return true;
    tools::logger()->warn("[Gimbal] Incomplete NavToGimbalV2 write.");
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write NavToGimbalV2: {}", e.what());
  }
  reconnect_requested_ = true;
  return false;
}

bool Gimbal::write_packet(DecisionToGimbal packet)
{
  try {
    if (write_serial_packet(serial_, packet)) return true;
    tools::logger()->warn("[Gimbal] Incomplete DecisionToGimbal write.");
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write DecisionToGimbal: {}", e.what());
  }
  reconnect_requested_ = true;
  return false;
}

void Gimbal::reconnect()
{
  std::lock_guard<std::mutex> reconnect_lock(reconnect_mutex_);
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      serial::Timeout timeout(10, 50, 0, 50, 0);
      serial_.setTimeout(timeout);
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
