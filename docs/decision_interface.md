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
| `/sentry/offline_info` | `SentryInfoOffline` | 待 MCU 上行帧扩展 |
| `/sentry/radar_info` | `RadarInfo` | 待 MCU/雷达 |

`Publish2DecisionMaking` 每 10ms 检查一次 `RefereeState.sequence`，有新帧才发布。

## 下行（决策发布，auto-aim 订阅）

| 话题 | 消息 | 处理 |
| --- | --- | --- |
| `/sentry/decision_command` | `DecisionCommand` | `Subscribe2Decision` 接收；待 MCU 决策下行帧确定后打包下发 |

下行统一为「带 `request_id` 的动作」，`mode` 显式选择 `ONE_SHOT` / `POLLED`，`POLLED` 带 `interval_ms`。
MCU 执行后应回 `DecisionAck`（`/sentry/decision_ack`），由 auto-aim 发布。

## 与串口帧的关系

- 现有高频控制帧（`VisionToGimbal` / `NavToGimbalV2`）**保持不变**。
- 新增的裁判/决策帧是低频的、带 `version` 与 `crc16` 的统一帧，仅承载上述消息的数据。
  字节布局与动作集合待与电控/MCU 确认后实现。

## 构建依赖

auto-aim 是纯 CMake 工程，ROS 为可选依赖（`SP_VISION_WITH_ROS2`）。使用本接口需要
`find_package(sentry_interfaces)`，即运行/编译环境需 source 含该包的 ROS 工作区
（当前该包在决策仓库 `sentry_decision_RM27/src/sentry_interfaces`）。

## 当前进度

- [x] 上行迁到 5 个 `sentry_interfaces` 消息（Phase 1：GameInfo / SentryInfoOnline / TeamInfo 有数据）。
- [x] 新增 `Subscribe2Decision` 接收 `/sentry/decision_command`（暂只记录，待 MCU 帧）。
- [ ] MCU 上行帧扩展，填充 Offline / Radar / 更多字段。
- [ ] MCU 下行帧与 `DecisionAck`。
