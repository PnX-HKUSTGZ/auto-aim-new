# 自瞄世界系可视化

`auto_aim_debug_mpc` 的 `/auto_aim/targets` 发布三维场景，
`/auto_aim/transforms` 发布 `world → gimbal` 变换，
`/auto_aim/camera_transform` 发布 `gimbal → camera` 固定外参。
Foxglove 3D 面板的 Fixed frame 和 Display frame 均选择 `world`。

世界系定义：初始 IMU 姿态作为世界系参考，经 `R_gimbal2imubody` 的逆换基得到。
坐标单位为米，原点为云台参考原点，未加入底盘平移。

电控四元数 `q` 表示“初始姿态到当前姿态”的旋转
`R_initial2current = R(q)`。令 `A = R_gimbal2imubody`：
`R_gimbal2world = A.transpose() * R(q) * A`。
standard3 使用行列式 +1 的合法旋转矩阵：
`A = [0, 1, 0; -1, 0, 0; 0, 0, 1]`
（gimbal X → IMU -Y、gimbal Y → IMU X、gimbal Z → IMU Z），
yaw 绕固定世界 Z、pitch 绕 gimbal 局部 X。
Foxglove `FrameTransform.rotation` 表示子坐标系在父坐标系中的姿态，
`world -> gimbal` 直接发布 `R_gimbal2world` 对应的四元数。
standard3 相机外参已换基，理想映射为
`(x_gimbal,y_gimbal,z_gimbal) = (x_camera,z_camera,-y_camera)`。
两套手眼标定程序的理想轴矩阵同步采用此定义。
相机外参组成 `R_camera2world = R_gimbal2world * R_camera2gimbal`，
其中 `R_camera2gimbal` 表示相机系到云台系的旋转。

- `observed_armor_*`：青色的当帧观测装甲板，位置和姿态来自 Solver 的
  `xyz_in_world`、`ypr_in_world`；文字显示世界系 XYZ。尚无跟踪目标时也显示。
- `auto_aim_target_*`：EKF 估计的目标中心和装甲板，绿色表示匹配装甲板，
  黄色表示其余推算装甲板。
- `gimbal_attitude`：实体绑定到 `gimbal` frame，并由每帧发布的
  `world → gimbal` 变换驱动；红、绿、蓝轴分别表示云台局部 X、Y、Z。
- `camera_frame`：实体绑定到 `camera` frame，显示相机局部 X 向右、Y 向下、
  Z 沿光轴向前；其位姿来自 `R_camera2gimbal` 和 `t_camera2gimbal`。
- `ballistic_trajectory`：世界系中的弹道与落点。

Tracker 在目标初始化或更新前批量解算候选装甲板，每帧只解算一次。
场景直接使用世界系观测，不再乘云台旋转矩阵；PnP 失败或被判定为
`not_armor` 的观测不显示。每帧场景替换上一帧，消失的观测不会残留。

如果固定目标随云台转动，启动时添加 `--frame-debug=true`。
程序每秒输出一条 `[WorldFrame]`，包含图像时刻的四元数、解算的云台世界姿态、
最新电控反馈角原始值（`feedback_*_raw`，不预设单位），以及首块未被拒绝的候选装甲板的云台系/世界系坐标。
没有候选时 `observed_valid=false`，仍输出姿态信息；`imu_q_valid` 标记四元数有效性。
保持同一个目标可见，分别在两个云台角度停稳后记录日志，可以检查旋转补偿方向。
电控反馈角是最新状态，不保证与图像时刻完全同步；停稳后对照可减少时差影响。

高度漂移排查：`[WorldFrame]` 同时输出 `observed_z_world`、
`observed_z_inverse_rotation`（仅诊断，不参与控制）、`ekf_z`、`ekf_vz`、
`ekf_h1`、`ekf_h2` 和 `matched_armor_id`。对固定的同一块装甲板，
分别在水平与抬头姿态停稳后比较；多目标时首块观测不一定是 EKF 匹配目标，
应先用单一目标排查。若停稳后仍漂移，不能仅归因于图像/姿态时间差。
