# 下位机通信协议（auto-aim ↔ MCU）

> 与 MCU 对齐的是第 2 节的**新帧族 v1**；第 1 节现有帧保持不变，仅作背景。
> ROS 侧契约见决策仓库 `docs/INTERFACES.md`；位段含义见 `sentry_decision_core/referee_protocol.hpp`。

## 1. 约定与现有帧

- 物理层：`/dev/ttyACM0` @ 115200 8N1；串口由 auto-aim 独占，决策 / 导航不直接接触。
- 字节序：全部小端。
- 现有帧（**不变**）：

| 头 | 方向 | 大小 | 用途 |
| --- | --- | --- | --- |
| `0x5A` | MCU → 上位机 | 54 B | IMU / 云台 + 裁判基础字段（血量 / 弹量 / `game_start` 等） |
| `0xA5` | 上位机 → MCU | 28 B | 云台控制 |
| `0xA6` | 上位机 → MCU | 28 B | 底盘速度 |

- 旧 `0xA7`（4 B，仅 `ifreload`）已废弃，由 §2.3 取代。
- 新帧以 `0xAA` 开头，与 `0x5A` 帧按首字节区分、共用同一串口。

## 2. 目标协议 v1（需要 MCU 实现）

### 2.1 帧格式

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | uint8 | `magic` | `0xAA` |
| 1 | uint8 | `version` | `0x01` |
| 2 | uint8 | `type` | `0x01` 上行裁判 / `0x02` 下行决策 / `0x03` 上行回执 |
| 3 | uint16 | `length` | payload 字节数 |
| 5 | uint8[length] | `payload` | 见 §2.2–§2.4 |
| 5+length | uint16 | `crc16` | 小端 |

- **CRC-16/MCRF4XX**：poly `0x1021`（反射 `0x8408`）、init `0xFFFF`、输出不异或、小端；
  覆盖 `version` 到 payload 末字节（偏移 `1 ~ 4+length`）。
- 帧示例（裁判上行，payload 29 B）：`AA 01 01 1D 00 <29 B payload> <crc_lo> <crc_hi>`。

### 2.2 `type=0x01` 上行：裁判 / 自身（建议 10 Hz，payload 29 B）

| 偏移 | 类型 | 字段 | 单位 / 说明 |
| --- | --- | --- | --- |
| 0 | uint8 | `game_status` | 0 未开始 / 1 准备 / 2 自检 / 3 倒计时 / 4 比赛中 / 5 结算 |
| 1 | uint16 | `game_time_remaining` | s |
| 3 | uint16 | `coin_remaining` | 己方金币 |
| 5 | uint8 | `detect_color` | 红蓝方，编码待确认 |
| 6 | uint8 | `flags` | bit0 `can_rebuild_outpost`，其余保留填 0 |
| 7 | uint16 | `sentry_hp` | 己方哨兵血量 |
| 9 | uint16 | `sentry_ammo` | 剩余发弹量 |
| 11 | uint16 | `our_base_hp` | 己方基地血量 |
| 13 | uint16 | `our_outpost_hp` | 己方前哨血量 |
| 15 | uint16 | `enemy_base_hp` | 敌方基地血量 |
| 17 | uint16 | `enemy_outpost_hp` | 敌方前哨血量 |
| 19 | uint32 | `event_code` | 0x0101 场地事件原始值 |
| 23 | uint32 | `sentry_info_1` | 0x020D 兑换 / 复活原始值 |
| 27 | uint16 | `sentry_info_2` | 0x020D 脱战 / 姿态原始值 |

- `event_code` / `sentry_info_1` / `sentry_info_2`：MCU **只转发原始整数，不解析**，位段含义在决策侧。
- 本版暂不发送：`sentry_info_3`、雷达、队友位置、手动点等。

### 2.3 `type=0x02` 下行：决策命令（payload 12 B）

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | uint32 | `request_id` | 唯一请求号，回执原样返回 |
| 4 | uint8 | `kind` | 动作类型，见下表 |
| 5 | uint8 | `mode` | 0 一次性 / 1 轮询（电平） |
| 6 | uint16 | `interval_ms` | 仅 `mode=1`，重发间隔 |
| 8 | int32 | `value` | 数量 / 次数 |

| `kind` | 动作 | 说明 |
| --- | --- | --- |
| 1 | 本地兑换发弹量 | `value` = 数量；需已占领增益点 |
| 2 | 本地兑换血量 | `value` = 数量 |
| 3 | 确认免费复活 | 电平 |
| 4 | 兑换立即复活 | 电平 |
| 5 | 远程兑换发弹量 | `value` = 次数；需脱战 |
| 6 | 远程兑换血量 | `value` = 次数；需脱战 |

执行规则：

- **`mode=0` 一次性**：按 `request_id` 去重，只执行一次；重复 `request_id` 忽略。
- **`mode=1` 轮询**：以最新一条为准；`kind=3/4` 对应裁判 `0x0120` bit0 / bit1，收到即置位，
  超时未再收到（建议 3×`interval_ms`）即清零，避免单帧丢失。

### 2.4 `type=0x03` 上行：执行回执（payload 6 B）

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | uint32 | `request_id` | 对应下行 |
| 4 | uint8 | `accepted` | 0 拒绝 / 1 成功 |
| 5 | uint8 | `code` | 0 = 成功，其余待约定 |

- 一次性动作执行后回一条；轮询动作仅在状态变化时回，避免刷屏。

## 3. 与 MCU 对齐清单

- [ ] `detect_color` 编码（0 / 1 分别代表哪方）。
- [ ] `DecisionAck.code` 取值表；`interval_ms` 与超时取值。
- [ ] MCU 能否直接转发裁判 `0x020D` 原始位段；`event_code` 是否有现成来源。
- [ ] 两类帧共存的发送节拍（建议裁判帧 10 Hz，独立于 100 Hz 控制帧）。
- [ ] 是否需要 `sentry_info_3` / 电容 / 热量（后续版本再加）。