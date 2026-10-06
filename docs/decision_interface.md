# 决策接口（auto-aim ↔ 决策）

> 权威来源是决策仓库的 `sentry_interfaces` 消息与 `docs/INTERFACES.md`；本文从 auto-aim 视角说明它在链路中的角色与改动点。

## 角色

auto-aim 是上位机与 MCU 的**通信网关**：串口（`/dev/ttyACM0` @ 115200，`io::Gimbal`）由 auto-aim 独占，
决策/导航不直接接触串口。与决策的交互全部发生在 ROS 图上。

```text
MCU  <--串口-->  auto-aim (io::Gimbal)
                     │  ROS (sentry_interfaces)
                     ▼
                 决策 (RosIoNode)
```

## 上行（auto-aim 发布，决策订阅）

| 话题 | 消息 | 数据来源 |
| --- | --- | --- |
| `/sentry/game_info` | `GameInfo` | `Gimbal::referee_state()` |
| `/sentry/online_info` | `SentryInfoOnline` | 同上 |
| `/sentry/team_info` | `TeamInfo` | 同上 |
| `/sentry/offline_info` | `SentryInfoOffline` | 自瞄视觉 / 本机状态（`is_get`/`armor_pos`/`armor_num` 待接入） |

`Publish2DecisionMaking` 每 10ms 检查一次 `RefereeState.sequence`，有新帧才发布。

## 下行（决策发布，auto-aim 订阅）

| 话题 | 消息 | 处理 |
| --- | --- | --- |
| `/sentry/decision_command` | `DecisionCommand` | `Subscribe2Decision` 接收；待 MCU 决策下行帧确定后打包下发 |

下行统一为「带 `request_id` 的动作」，`mode` 显式选择 `ONE_SHOT` / `POLLED`，`POLLED` 带 `interval_ms`。
MCU 执行后应回 `DecisionAck`（`/sentry/decision_ack`），由 auto-aim 发布。

## 与串口帧的关系

- **当前**串口协议见 [docs/serial_protocol.md](serial_protocol.md)。
- 现有高频控制帧（`VisionToGimbal` / `NavToGimbalV2`）**保持不变**。
- 旧的 4 字节 `DecisionToGimbal`（`ifreload`）已无 ROS 调用方（原 `/set_bool` service 已移除），
  待 P2.3b 用统一帧替换。
- 新帧为低频、带 `version`/`type`/`length` 与 `crc16` 的统一帧；v1 字节布局、动作 `code` 与电平语义见 [serial_protocol.md](serial_protocol.md) §2。

## 构建依赖

auto-aim 是纯 CMake 工程，ROS 为可选依赖：找不到 ROS 时整体跳过 ROS 代码（`SP_VISION_WITH_ROS2`）。
一旦编译 ROS，`sentry_interfaces` 就是**硬依赖**（`find_package(sentry_interfaces REQUIRED)`，缺则配置失败），
避免只跳过决策桥、却把导航桥一起悄悄编译掉。该包在决策仓库 `sentry_decision_RM27/src/sentry_interfaces`。

## 当前进度

- [x] 上行：`GameInfo` / `SentryInfoOnline` / `TeamInfo` 有数据；`Offline` 的视觉字段待在 auto-aim 侧接入。
- [x] 下行：`Subscribe2Decision` 接收 `/sentry/decision_command`（当前仅记录，未写串口）。
- [ ] 下行串口帧与 `DecisionAck`（P2.3b，待与电控/MCU 定协议）。
- [ ] 上行帧扩展（`Offline` 视觉字段、雷达等后续再加）。
