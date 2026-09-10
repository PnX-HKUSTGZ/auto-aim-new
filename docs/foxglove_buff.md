# 能量机关 Foxglove 三维调试

`auto_buff_debug_mpc` 内置 Foxglove WebSocket 服务，默认监听 `127.0.0.1:8765`。
通过 `/buff/scene` 发布 JSON 编码的 `foxglove.SceneUpdate`，同时通过 `/buff/transforms`
发布 `foxglove.FrameTransform`，建立 `world → camera` 坐标系树。
无需 ROS、Foxglove Bridge、URDF 或额外模型文件。
程序使用内置几何图元建模，消息 schema 已随代码保存并嵌入可执行文件。

## 编译和运行

在项目已有依赖的基础上安装 Boost 头文件（1.70 及以上；Ubuntu 22.04 的包满足要求）：

```bash
sudo apt install libboost-dev
cmake -S . -B build
cmake --build build --target auto_buff_debug_mpc -j2
./build/auto_buff_debug_mpc configs/standard4.yaml assets/demo/buff_video.mp4
```

请从项目根目录运行，并选择与视频相机标定、模型路径匹配的 YAML。
只需按原项目要求准备推理依赖；Foxglove 无需单独安装 C++ SDK。
日志出现 `Foxglove: ws://127.0.0.1:8765, topics /buff/scene + /buff/transforms, frame world` 表示服务已启动。

从另一台电脑连接时，运行：

```bash
./build/auto_buff_debug_mpc configs/standard4.yaml assets/demo/buff_video.mp4 \
  --foxglove-host=0.0.0.0 --foxglove-port=8765 --foxglove-prediction=0.2
```

连接地址使用运行程序那台机器的实际 IP，例如 `ws://192.168.1.10:8765`，不能填写 `0.0.0.0`。
两台机器之间需允许该 TCP 端口通信。
不需要三维输出时加 `--foxglove=false`；端口冲突时可改 `--foxglove-port=8766`。
`--foxglove-prediction` 是运动预览时间，单位秒，允许范围 `(0, 10]`，默认 `0.2`。

## Foxglove 面板设置

1. 打开 Foxglove，选择 **Open connection → Foxglove WebSocket**，填写
   `ws://127.0.0.1:8765`（远程连接替换 IP），点击连接。这里连接的是 Foxglove WebSocket 协议。
2. 添加 **3D** 面板，打开面板设置。在 **Frame** 中将 **Display frame** 和
   **Fixed frame** 都设为 `world`。程序通过 `/buff/transforms` 提供坐标系树；
   所有几何图元都已经转换到 `world`，无需另启 ROS TF 发布器。
3. 在 **Topics** 中打开 `/buff/scene` 的眼睛图标。如果配置过颜色/透明度覆盖，恢复消息自带颜色，
   确保话题总体透明度不为零。可展开实体，单独隐藏平面或参考模型。
4. 在 **View** 中打开 **3D view**。初次可将 **Distance** 设为 `8` 米、
   **Near** 设为 `0.01`、**Far** 设为 `100`，然后旋转和缩放视角寻找目标。
   鼠标滚轮缩放，右键拖动旋转，左键拖动平移。
5. 目标位于世界原点前方数米处。若画面只有原点坐标轴，可在 **View → Target**
   填入 R 标中心的 `x/y/z`，将 **Distance** 调到 `3` 米。
   R 标中心可在 **Raw Messages** 面板选择 `/buff/scene`，查找
   `entities` 中 `id="rune_plane"` 的 `spheres[0].pose.position`。
   相机对准平面侧边时只能看到一条窄线，需旋转至正面或斜视角。

可以再添加一个 **Raw Messages** 面板检查实体坐标及 `metadata`。
配置完后保存布局供后续调试复用。Foxglove 对场景消息和坐标系的要求见
[SceneUpdate 文档](https://docs.foxglove.dev/docs/sdk/schemas/scene-update)及
[3D 面板文档](https://docs.foxglove.dev/docs/visualization/panels/3d)。

## 场景含义

| 实体 ID | 显示内容 | 数据来源 |
| --- | --- | --- |
| `world_camera` | 原点坐标轴、相机位置及光轴；轴颜色为 X 红、Y 绿、Z 蓝 | `Solver::camera_pose()` |
| `rune_plane` | 蓝色半透明平面、R 标球、法向箭头和符叶中心轨道 | `InactiveTargets::power_rune_plane` 与共享 R 标中心 |
| `five_blade_reference` | 灰色五叶轮廓及连接臂，相邻 72° | 根据第一片观测符叶及平面推算的几何参考 |
| `observed_inactive_blades` | 红色球和编号，表示当前观测到的未激活符叶装甲中心 | `InactiveTargets::rune_pieces[].armor_center` |
| `tracked_target` | 黄色球表示追踪器选中的装甲中心；黄色箭头表示零相位方向 | `RuneTarget` |
| `motion_prediction` | 绿色曲线及终点球，表示未来指定时长内的运动 | `predict_armor_position()`，使用追踪器的小符/大符运动模型 |

灰色参考符叶并不表示五片都被识别到，也不代表其激活状态。当前检测接口只输出未激活符叶的三维位置。
菱形轮廓沿径向和切向的半尺寸为 `0.127 m`，用于调试定位，不是完整机械 CAD。
蓝色平面是有限尺寸的薄片，用于显示拟合平面的朝向；薄片厚度只是显示效果。
绿色预测是运动预览，没有叠加瞄准器内部的弹丸飞行时间，不能当作 MPC 的最终弹着点。

所有位置单位为米。`world` 沿用算法坐标系：X 向前、Y 向左、Z 向上；相机局部坐标为 X 向右、Y 向下、Z 沿光轴向前。
当前离线程序使用单位云台四元数，世界姿态基于这个假设及 YAML 中的相机外参。
视频不包含真实 IMU 姿态时，显示结果也不会包含录制时云台的实际转动。

拟合平面在运动追踪器初始化前即可显示；黄色追踪结果和绿色预测在追踪器输出有效目标后出现。
每条消息都是完整快照，先清除旧实体再添加当前实体，目标丢失和符叶数量变化不会留下历史符叶。
网络线程最高约 30 Hz 发送最新快照，不限制离线推理速度；慢客户端可能跳过中间帧。
Foxglove 的消息时间是 UNIX 实时时间，算法预测仍使用视频帧时间，视频循环播放时预测时序保持原逻辑。
程序退出或数据源断开后，Foxglove 可能保留最后一帧供查看。

## 显示异常排查

- **`/buff/scene` 显示“没有数据”**：先确认调试程序仍在运行，且 Foxglove 连接的是启动日志中的
  IP 和端口。停止程序后，面板可能仍列出旧话题，但不会再收到数据。启动程序后重新连接，
  在 Raw Messages 的话题下拉框选择 `/buff/scene`（末尾没有 `/`），检查是否收到 `entities`。
  未识别到符叶时也会发送 `world_camera` 坐标轴，所以“没有数据”与“没有检测到符叶”是不同问题。
  若 Raw Messages 有数据而 3D 空白，再检查 `/buff/scene` 的眼睛图标、`world` 坐标系和视角。
- **No coordinate frames found，无法选择 world**：旧版本只在实体中写入 `frame_id="world"`，
  没有发布坐标变换，Foxglove 因而没有可选坐标系。重新编译、退出旧程序并启动新程序后，
  在 Foxglove 中断开并重新连接。确认数据源有 `/buff/transforms`，在 Raw Messages 中选择它，
  应能看到 `parent_frame_id: world`、`child_frame_id: camera` 和持续更新的时间戳。
  若当前面板有变换话题订阅开关，确保启用该话题。再将 Fixed frame / Display frame 设为 `world`。
  该变换由相机外参和云台姿态生成，即使没有检测到符叶也会持续发布。
  仅给实体填写 `frame_id` 不能替代坐标变换消息，参见
  [FrameTransform 定义](https://docs.foxglove.dev/docs/sdk/schemas/frame-transform)。
- **连接失败**：确认程序仍运行、日志没有端口占用错误；远程监听须设为 `0.0.0.0`，
  客户端填写服务器实际 IP。若浏览器报告不允许该 `ws://` 连接，可使用 Foxglove 桌面端。
- **话题存在但模型不显示**：在 Raw Messages 检查是否有 `rune_plane`。
  只有 `world_camera` 通常表示尚无有效 PnP 观测，先检查视频识别窗口、模型、相机标定和检测阈值。
  若已有实体，则检查 `/buff/scene` 可见性、`world` 坐标系、相机目标位置、3D 模式和裁剪距离。
- **平面方向或距离不对**：检查视频分辨率对应的内参、畸变和相机到云台外参；
  Foxglove 直接显示算法计算结果，不会补偿错误标定。重叠或遮挡时可隐藏 `rune_plane` 实体。
- **没有黄色/绿色结果**：观测可能尚不足以初始化追踪器；红色符叶和平面可以先于追踪结果出现。

## 验证

```bash
cmake --build build --target rune_scene_test -j2
./build/rune_scene_test
python3 tests/foxglove_websocket_test.py ./build/rune_scene_test
```

几何测试使用倾斜平面检查轨道与五叶共面、平面四元数、预测方向及丢失清理。
WebSocket 测试检查独立订阅变换时的坐标系发现、变换先于场景发送、消息格式、订阅 ID、取消订阅和重连；仅使用 Python 标准库。
