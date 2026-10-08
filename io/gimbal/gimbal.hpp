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

// ---- 统一帧（协议 v1，见 docs/serial_protocol.md §2）----
constexpr uint8_t kFrameMagic = 0xAA;
constexpr uint8_t kFrameVersion = 0x01;
constexpr uint8_t kFrameTypeRefereeUplink = 0x01;
constexpr uint8_t kFrameTypeDecisionCommand = 0x02;
constexpr uint8_t kFrameTypeDecisionAck = 0x03;
constexpr uint16_t kFrameMaxPayload = 64;

// type=0x01：MCU 上行裁判 / 自身（29 字节）。
struct __attribute__((packed)) RefereeUplinkPayload
{
  uint8_t game_status = 0;
  uint16_t game_time_remaining = 0;
  uint16_t coin_remaining = 0;
  uint8_t detect_color = 0;
  uint8_t flags = 0;  // bit0: can_rebuild_outpost
  uint16_t sentry_hp = 0;
  uint16_t sentry_ammo = 0;
  uint16_t our_base_hp = 0;
  uint16_t our_outpost_hp = 0;
  uint16_t enemy_base_hp = 0;
  uint16_t enemy_outpost_hp = 0;
  uint32_t event_code = 0;
  uint32_t sentry_info_1 = 0;
  uint16_t sentry_info_2 = 0;
};
static_assert(sizeof(RefereeUplinkPayload) == 29, "RefereeUplinkPayload layout mismatch");
static_assert(std::is_trivially_copyable_v<RefereeUplinkPayload>);

// type=0x02：上位机下行决策命令（12 字节）。
struct __attribute__((packed)) DecisionCommand
{
  uint32_t request_id = 0;
  uint8_t kind = 0;
  uint8_t mode = 0;  // 0: one-shot / 1: polled
  uint16_t interval_ms = 0;
  int32_t value = 0;
};
static_assert(sizeof(DecisionCommand) == 12, "DecisionCommand layout mismatch");
static_assert(std::is_trivially_copyable_v<DecisionCommand>);

// type=0x03：MCU 上行执行回执（6 字节）。
struct __attribute__((packed)) DecisionAck
{
  uint32_t request_id = 0;
  uint8_t accepted = 0;
  uint8_t code = 0;
};
static_assert(sizeof(DecisionAck) == 6, "DecisionAck layout mismatch");
static_assert(std::is_trivially_copyable_v<DecisionAck>);

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

  // 统一帧（协议 v1）字段；未收到统一帧时保持默认。
  bool has_unified = false;
  uint8_t game_status = 0;  // 0 未开始 / 1 准备 / 2 自检 / 3 倒计时 / 4 比赛中 / 5 结算
  uint16_t game_time_remaining = 0;
  uint16_t coins = 0;
  uint32_t event_code = 0;
  uint32_t sentry_info_1 = 0;
  uint16_t sentry_info_2 = 0;
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

  // 下行决策命令：入队后由 tx 线程按协议 v1 组帧发送。队列满返回 false。
  bool send_decision_command(const DecisionCommand & command);

  // 取出自上次调用以来收到的执行回执；无则返回 false。
  bool pop_decision_ack(DecisionAck * out);

private:
  using Clock = std::chrono::steady_clock;

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
  std::deque<DecisionCommand> decision_cmd_queue_;  // 由 tx_mutex_ 保护
  std::deque<DecisionAck> ack_queue_;                // 由 state_mutex_ 保护

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
  void read_unified_frame();
  void handle_unified_frame(uint8_t type, const uint8_t * payload, uint16_t length);
  void tx_loop();
  bool write_packet(VisionToGimbal packet);
  bool write_packet(NavToGimbalV2 packet);
  bool write_unified_frame(uint8_t type, const uint8_t * payload, uint16_t length);
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
