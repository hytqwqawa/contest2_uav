# Demo8 · EGO 终点方向约束 V1（未实飞验收）

状态：**开发版 / 静态测试通过不等于实飞放行**。本目录是 Demo7 冻结验收包的**独立副本**。`runtime/`、原 `demo/07_demo5_live_qr_v1_accepted/` 均不修改。不重新开展旧故障排查。

## 功能

- 保留 Demo7 的 D435i + QR 只读识别、POINT/EGO 航点、EGO B-spline 避障/重规划、起飞/降落流程、安全门控与围栏。
- 航点中的 `yaw_deg` **只允许在 EGO 航点**，可省略。使用 Demo Frame：沿 F 为 `0°`，左转为正（`+90°` 朝 L），允许 `[-180,180]`，转换为 World/ENU `atan2(Fy,Fx) + yaw_deg`。
- EGO 沿原 TANGENT yaw 避障飞到位置附近；在位置/速度稳定 0.80 秒后，利用原 `ego_trajectory_bridge.py` 的 `FIXED` 模式：抓取当前实测 XYZ，保持 XYZ，V/A=0，受 yaw 速率/超前限制，转至期望 yaw。
- 进入终点 FIXED 前，额外要求 XY < 0.12m、Z < 0.08m、速度 < 0.10m/s 且稳定 0.80 秒；不带 yaw 的 EGO 航点维持 Demo7 原判据。
- 在 EGO MUX 激活、健康/CH7/护栏许可持续有效、EGO 控制流实时的条件下，核实 bridge 输出的 P/V/A、yaw、`trajectory_interface` 的 `TRAJECTORY_ACTIVE` 状态、MAVROS 接收的实际 setpoint，并用 PX4 里程计实测满足 XY < 0.15m、Z < 0.10m、速度 < 0.12m/s、角度误差 <= 5° 且稳定 >= 1 秒才宣布该航点完成；最大漂移限制；总超时仍受航段 `segment_timeout_s` 约束。
- 通过既有 `disable_trajectory → abort_trajectory → use_point → enable_trajectory` 恢复 POINT HOLD（实际姿态由 POINT abort 捕获），再进入下一航点；最后一航点才调用原降落链。
- `yaw_deg` 省略则按 Demo7 原 TANGENT 到点流程，不做转向。POINT 始终保持航段起始 yaw，**没有避障**。

## 航点示例（待现场确认，非建议直接实飞）

编辑 **本 Demo8 目录内** `demo5_waypoints.yaml`：

```yaml
waypoints:
  - {forward: 1.000, left: 1.700, mode: POINT}
  - {forward: 3.000, left: 0.500, mode: EGO, yaw_deg: 0.0}
  - {forward: 4.500, left: 1.800, mode: EGO}
```

样例 P2 在到点后转向“本次 Demo Frame 正前方”；P3 保持原 Demo7 行为并降落。**改变 yaw 时，务必地面检查周围净空、线缆、视场与任务要求**。这里的约束是**到点后的机头朝向**，不是 EGO B-spline 的入射运动方向，也不是同时优化 yaw 的四维规划。

## 操作分级

**【只读／离线，不启动 ROS】**

```bash
cd ~/contest2_uav/demo/08_ego_terminal_yaw_v1
bash check_demo8_static.sh
```

**【地面｜CH5 DISARM，CH6 POSCTL，CH7 LOW，CH8 SAFE；仅传感器／健康只读检查】**

在已按原流程启动 ROS 后，查询（不启动任何任务）：

```bash
source ~/contest2_uav/runtime/env.sh
rostopic info /contest2/ego_uav_interface/yaw_mode
rostopic info /contest2/ego_uav_interface/yaw_target_deg
rostopic info /contest2/ego_uav_interface/ego_bridge_status
rostopic info /contest2/ego_uav_interface/ego_setpoint
rostopic info /contest2/ego_uav_interface/trajectory_interface_status
rostopic info /contest2/mavros/setpoint_raw/local
```

**【实飞｜禁止直接试运行本开发版】** 现有 Demo7 的三个终端入口仅作为参考，本 Demo8 准备了对应 `start_core.sh`、`start_backend.sh`、`run_demo8.sh`，但它们能启动真正的飞行系统。必须先进行独立安全复核、地面联调、现场净空和飞手许可；保持 CH5 物理 ARM/DISARM、CH6 POSCTL 飞手接管、CH7 LOW/MID/HIGH、CH8 急停保护。不得将静态检查通过当成已验证实飞。

## 架构前提

- 本机 `ego_replan_fsm.cpp` 接受 `PoseStamped.position.x/y/z`，不使用 orientation。无需修改冻结的 EGO 优化器。
- 本机 `traj_server.cpp` 在轨迹结束后依然周期发布悬停 `PositionCommand`；原 bridge 的 `FIXED` 依赖这些消息驱动输出。若运行中中断，现有 MUX/trajectory_interface 会拒绝过期命令，Demo8 对过期也立即报错。
- 本 Demo8 **不会新建 MAVROS setpoint publisher，也不会执行软件 ARM/DISARM**；复用已验收控制链路。
- 正式部署前还要现场核对实际 `runtime` 的 bridge 是否与交接源码一致，另外确保 `yaw_tangent_demo.rviz` 等 Demo7 原附件存在。
