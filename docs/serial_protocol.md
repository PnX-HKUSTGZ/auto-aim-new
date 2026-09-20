# 下位机串口协议（auto-aim ↔ MCU）

> 现状说明：此前该协议只隐式定义在 `io/gimbal/gimbal.hpp` 的 C++ 结构体里，没有文档。本文补齐**当前在用**的协议。
> 上位机内部（ROS）契约见 `docs/decision_interface.md` 与决策仓库 `docs/INTERFACES.md`。

## 1. 物理层

- 连接：MicroUSB 虚拟串口（新）/ USB2CAN（旧）。
- 默认：`/dev/ttyACM0`（可用 udev 固定为 `/dev/gimbal`），`baud_rate: 115200`，8N1。
- 由 `io::Gimbal` 独占；决策/导航不直接接触串口。

## 2. 帧格式

每帧 = **1 字节包头 Magic Number** + 一个 `__attribute__((packed))` 结构体，结构体末尾是 `uint16_t crc16`。

- 魔数区分帧类型：`0x5A` 上行、`0xA5`/`0xA6`/`0xA7` 下行。
- 所有结构体都用 `static_assert` 锁定大小。
- 位域（`GimbalToVision` 第 2 字节）按 GCC 小端、低位在前的顺序打包。

**CRC16**（`tools/crc.hpp`）：

| 参数 | 值 |
| --- | --- |
| 宽度 | 16 |
| 多项式 | `0x1021`（反射形式 `0x8408`） |
| 初值 | `0xFFFF` |
| 输入/输出反射 | 是 / 是 |
| 最终异或 | `0x0000` |
| 常见命名 | CRC-16/MCRF4XX（RM 圈常俗称 CRC16） |
| 字节序 | **小端**（低字节在前） |

- 发送：`crc16 = get_crc16(帧首, 帧长 - 2)`，覆盖除末尾 `crc16` 外的全部字节。
- 接收：`check_crc16(帧首, 帧长)`，读取末尾两字节小端并与重算值比较。

## 3. 上行帧 `GimbalToVision`（MCU → header 0x5A，共 54 字节）

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | uint8 | `header` | 固定 `0x5A` |
| 1 | bits | `detect_color:1` | 红/蓝方 |
| 1 | bits | `reset_tracker:1` | 复位跟踪 |
| 1 | bits | `mode:2` | 0 空闲 / 1 自瞄 / 2 小符 / 3 大符 |
| 1 | bits | `game_start:1` | 比赛是否开始 |
| 1 | bits | `can_rebuild_outpost:1` | 是否可重建前哨 |
| 1 | bits | `reserved:2` | 保留 |
| 2 | uint16 | `sentryHP` | 哨兵血量 |
| 4 | uint16 | `our_baseHP` | 己方基地血量 |
| 6 | uint16 | `enemy_baseHP` | 敌方基地血量 |
| 8 | uint16 | `our_outpostHP` | 己方前哨血量 |
| 10 | uint16 | `enemy_outpostHP` | 敌方前哨血量 |
| 12 | uint16 | `remain_ammo` | 剩余发弹量 |
| 14 | float[4] | `q[4]` | IMU 四元数，**wxyz 顺序** |
| 30 | float | `yaw` | 云台 yaw |
| 34 | float | `yaw_vel` | |
| 38 | float | `pitch` | 云台 pitch |
| 42 | float | `pitch_vel` | |
| 46 | float | `bullet_speed` | 弹速 |
| 50 | uint16 | `bullet_count` | 累计弹丸数 |
| 52 | uint16 | `crc16` | |

`Gimbal::read_thread()` 逐字节找 `0x5A` 头 → 读满 54 字节 → 校验 CRC → 解析为 `io::RefereeState`（`sequence` 递增）与云台状态。

## 4. 下行帧

### 4.1 `VisionToGimbal`（云台控制，0xA5，28 字节）

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | uint8 | `header` | `0xA5` |
| 1 | uint8 | `mode` | 0 不控制 / 1 控云台不开火 / 2 控云台且开火 |
| 2/6/10 | float | `yaw`, `yaw_vel`, `yaw_acc` | |
| 14/18/22 | float | `pitch`, `pitch_vel`, `pitch_acc` | 发送时取负 |
| 26 | uint16 | `crc16` | |

### 4.2 `NavToGimbalV2`（底盘速度，0xA6，28 字节）

| 偏移 | 类型 | 字段 |
| --- | --- | --- |
| 0 | uint8 | `header` = `0xA6` |
| 1 | uint8 | `follow_mark` |
| 2/6/10 | float | `linear_x`, `linear_y`, `linear_z` |
| 14/18/22 | float | `angular_x`, `angular_y`, `angular_z` |
| 26 | uint16 | `crc16` |

由 `/cmd_vel`（导航）驱动，`linear_x/y` 会乘 `cmd_vel_linear_scale`（配置项）。

### 4.3 `DecisionToGimbal`（决策，0xA7，4 字节）

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | uint8 | `header` | `0xA7` |
| 1 | uint8 | `ifreload` | **唯一内容字段，语义未确认** |
| 2 | uint16 | `crc16` | |

## 5. 发送调度（`Gimbal::tx_loop`）

- 云台帧：上限 `vision_tx_max_hz`（默认 100 Hz）；`vision_timeout_ms`（默认 100）无更新则发一帧停止帧。
- 导航帧：上限 `nav_tx_max_hz`（默认 50 Hz）；`nav_timeout_ms`（默认 200）无更新则发停止帧。
- 决策帧：请求入队（`decision_queue_capacity` 默认 16），逐条发送；`send_decision` 等待写入结果，超时默认 1s。
- 三者共享一条串口，按最小周期与优先级在 tx 线程内串行发送。

## 6. 已知缺口（本次重构要解决）

1. **决策下行过窄**：只有 `ifreload` 一个字节，复活/兑换/远程兑换等无法下发。
2. **无回执**：`send_decision` 只返回串口写入结果，不是 MCU 执行确认。
3. **无版本/长度字段**：结构体一旦改就两边必须同步重编。
4. `ifreload` 语义未确认（建议暂不依赖）。
5. 位域布局依赖编译器，跨工具链需注意。

## 7. 目标：统一、版本化的裁判/决策帧

见决策仓库 [docs/INTERFACES.md](https://github.com/PnX-HKUSTGZ/sentry_decision_RM27/blob/main/docs/INTERFACES.md)：
统一为 `[magic][version][type][length][payload][crc16]`，payload 为版本化结构体，只承载裁判/决策数据（低频）；
现有高频控制帧 `VisionToGimbal` / `NavToGimbalV2` 保持不变。字节布局与动作集合待确认。

## 8. 参考：minco 导航的 MCU 协议

`navi_minco_bit/src/navigation/communication/include/utils/` 下另有一套分帧协议：

- `protocol.hpp`：`PacketHeader { start1=0xA5, start2=0x5A, from, to, packet_type, data_len, checksum }`，
  校验为**逐 2 字节求和**（非 CRC16），且 `check()` 目前被打桩为恒真（未完成）。
- `custom_protocol.hpp`：`GameInfo` / `SentryInfoOnline` / `SentryInfoOffline` / `TeamInfo` / `RadarInfo` /
  `BehaviorData` 等数据体，字段与我们 `sentry_interfaces` 的消息一一对应。

它属于 minco 导航栈，与 auto-aim 哨兵链路是**两套并行协议**。可作为新统一帧的设计参考（包头/length/type 的思路一致）。
