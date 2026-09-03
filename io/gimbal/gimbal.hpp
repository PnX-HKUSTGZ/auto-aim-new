#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{
struct __attribute__((packed)) GimbalToVision
{
  uint8_t header = 0x5A;
  uint8_t detect_color : 1;
  uint8_t reset_tracker : 1;  // 哨兵没有手操
  uint8_t mode : 2;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  uint8_t game_start : 1;
  uint8_t can_rebuild_outpost : 1;  // 是否可以重建前哨站
  uint8_t reserved : 2;
  uint16_t sentryHP;
  uint16_t our_baseHP;
  uint16_t enemy_baseHP;
  uint16_t our_outpostHP;
  uint16_t enemy_outpostHP;
  uint16_t remain_ammo;  // 剩余发弹量
  float q[4];  // wxyz顺序，小yaw的q
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) == 54, "GimbalToVision layout mismatch");
static_assert(std::is_trivially_copyable_v<GimbalToVision>);

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t header = 0xA5;
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
};

static_assert(sizeof(VisionToGimbal) == 28, "VisionToGimbal layout mismatch");
static_assert(std::is_trivially_copyable_v<VisionToGimbal>);

struct __attribute__((packed)) NavToGimbalV2
{
  uint8_t header = 0xA6;
  uint8_t follow_mark = 0;

  float linear_x;
  float linear_y;
  float linear_z;

  float angular_x;
  float angular_y;
  float angular_z;

  uint16_t crc16 = 0;
};

static_assert(sizeof(NavToGimbalV2) == 28, "NavToGimbalV2 layout mismatch");
static_assert(std::is_trivially_copyable_v<NavToGimbalV2>);

struct __attribute__((packed)) DecisionToGimbal
{
  uint8_t header = 0xA7;
  uint8_t ifreload;
  uint16_t crc16 = 0;
};

static_assert(sizeof(DecisionToGimbal) == 4, "DecisionToGimbal layout mismatch");
static_assert(std::is_trivially_copyable_v<DecisionToGimbal>);

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

struct RefereeState
{
  uint64_t sequence = 0;
  uint8_t detect_color = 0;
  bool reset_tracker = false;
  bool game_start = false;
  bool can_rebuild_outpost = false;
  uint16_t sentry_hp = 0;
  uint16_t our_base_hp = 0;
  uint16_t enemy_base_hp = 0;
  uint16_t our_outpost_hp = 0;
  uint16_t enemy_outpost_hp = 0;
  uint16_t remain_ammo = 0;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  RefereeState referee_state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  void send(const VisionToGimbal & command);

  void send_navigation(
    float linear_x, float linear_y, float linear_z, float angular_x, float angular_y,
    float angular_z);

  uint8_t set_follow_mark(bool enabled);

  bool send_decision(bool reload, std::chrono::milliseconds timeout = std::chrono::seconds(1));

private:
  using Clock = std::chrono::steady_clock;

  struct DecisionRequest
  {
    DecisionToGimbal packet{};
    std::promise<bool> completion;
  };

  serial::Serial serial_;

  std::thread read_thread_;
  std::thread tx_thread_;
  std::atomic<bool> quit_ = false;
  std::atomic<bool> reconnect_requested_ = false;
  mutable std::mutex state_mutex_;
  std::mutex reconnect_mutex_;

  GimbalToVision rx_data_{};

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_{};
  RefereeState referee_state_{};
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  std::mutex tx_mutex_;
  std::condition_variable tx_cv_;
  VisionToGimbal latest_vision_{};
  NavToGimbalV2 latest_nav_{};
  bool vision_pending_ = false;
  bool nav_pending_ = false;
  bool has_vision_ = false;
  bool has_nav_ = false;
  bool vision_watchdog_sent_ = false;
  bool nav_watchdog_sent_ = false;
  Clock::time_point vision_updated_at_{};
  Clock::time_point nav_updated_at_{};
  Clock::time_point vision_next_send_at_{};
  Clock::time_point nav_next_send_at_{};
  std::deque<std::shared_ptr<DecisionRequest>> decision_queue_;

  Clock::duration vision_min_period_ = std::chrono::milliseconds(10);
  Clock::duration nav_min_period_ = std::chrono::milliseconds(20);
  Clock::duration vision_timeout_ = std::chrono::milliseconds(100);
  Clock::duration nav_timeout_ = std::chrono::milliseconds(200);
  std::size_t decision_queue_capacity_ = 16;

  double cmd_vel_linear_scale_ = 0.4;
  uint8_t follow_mark_ = 2;
  uint8_t follow_mark_default_value_ = 2;
  uint8_t follow_mark_start_value_ = 1;
  uint8_t follow_mark_rough_value_ = 0;
  int follow_mark_hold_nav_count_ = 10;
  int follow_mark_hold_remaining_ = 0;
  double follow_mark_zero_linear_scale_ = 0.5;

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void tx_loop();
  bool write_packet(VisionToGimbal packet);
  bool write_packet(NavToGimbalV2 packet);
  bool write_packet(DecisionToGimbal packet);
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP