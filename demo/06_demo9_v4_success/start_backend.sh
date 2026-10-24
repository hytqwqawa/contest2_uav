#!/usr/bin/env bash
# Terminal 2. Camera depth -> EGO planner -> POINT/EGO mux and bridge.
set -eo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HOME/contest2_uav"
source "$ROOT/runtime/env.sh"
export ROS_MASTER_URI=http://192.168.8.102:11311
export ROS_IP=192.168.8.102
export ROS_NAMESPACE=/contest2
export ROS_PACKAGE_PATH="$D/ros_src:${ROS_PACKAGE_PATH:-}"
rospack profile >/dev/null
PKG="$(rospack find ego_uav_interface)"
[ "$PKG" = "$D/ros_src/ego_uav_interface" ] || { echo "[ERROR] wrong ego_uav_interface package: $PKG"; exit 1; }
rospack find ego_planner >/dev/null || { echo '[ERROR] EGO-Planner package missing in runtime'; exit 1; }
LOGDIR="$ROOT/data/logs/demo10_compact/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOGDIR"
PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do kill -INT "${PIDS[$i]}" 2>/dev/null || true; done
  for pid in "${PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM
wait_topic() { timeout "${2:-30}" rostopic echo -n 1 "$1" >/dev/null 2>&1; }
wait_node() {
  local deadline=$((SECONDS + ${2:-20}))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if rosnode list 2>/dev/null | grep -Fxq "$1"; then return 0; fi
    sleep 0.4
  done
  return 1
}
timeout 8 rostopic list >/dev/null || { echo '[ERROR] ROS master not available; start Terminal 1'; exit 1; }
wait_topic /contest2/mavros/local_position/odom 60 || { echo '[ERROR] no PX4 odometry'; exit 1; }
# Do not block an entire backend because flight_healthy was momentarily False.
# Existing flight-critical interfaces and PX4 still enforce their requirements.
if rosnode list 2>/dev/null | grep -Eq '^/contest2/(point_traj_node|trajectory_mux|ego_trajectory_bridge|drone_0_ego_planner_node|drone_0_traj_server)$'; then
  echo '[ERROR] duplicate backend detected'; exit 1
fi
if rosnode list 2>/dev/null | grep -Eiq 'px4ctrl|px4_ctrl'; then
  echo '[ERROR] competing PX4Ctrl detected'; exit 1
fi
echo '[BACKEND] D435 depth camera...'
roslaunch realsense2_camera rs_camera.launch \
  serial_no:=939722070209 camera:=camera \
  enable_depth:=true enable_color:=true \
  color_width:=640 color_height:=480 color_fps:=30 \
  enable_infra1:=false enable_infra2:=false \
  enable_gyro:=false enable_accel:=false enable_pose:=false \
  depth_width:=640 depth_height:=480 depth_fps:=30 initial_reset:=false \
  diagnostics_topic:=/contest2/diagnostics \
  tf_topic:=/contest2/tf tf_static_topic:=/contest2/tf_static \
  >"$LOGDIR/d435.log" 2>&1 & PIDS+=("$!")
wait_topic /contest2/camera/depth/image_rect_raw 40 || { echo '[ERROR] no depth images'; tail -n 30 "$LOGDIR/d435.log"; exit 1; }
echo '[BACKEND] Camera extrinsic...'
roslaunch "$PKG/launch/camera_extrinsic.launch" >"$LOGDIR/extrinsic.log" 2>&1 & PIDS+=("$!")
wait_topic /contest2/vins_fusion/extrinsic 12 || { echo '[ERROR] no camera extrinsic'; exit 1; }
echo '[BACKEND] EGO planner...'
roslaunch "$PKG/launch/ego_uav2.launch" max_vel:=0.15 >"$LOGDIR/ego.log" 2>&1 & PIDS+=("$!")
wait_node /contest2/drone_0_ego_planner_node 30 || { echo '[ERROR] EGO planner did not start'; tail -n 40 "$LOGDIR/ego.log"; exit 1; }
wait_node /contest2/drone_0_traj_server 20 || { echo '[ERROR] traj_server did not start'; exit 1; }
echo '[BACKEND] POINT/EGO source mux and bridge...'
roslaunch "$D/uav2_demo5_multi_waypoint_interface.launch" >"$LOGDIR/interface.log" 2>&1 & PIDS+=("$!")
for node in point_traj_node trajectory_mux ego_trajectory_bridge; do
  wait_node "/contest2/$node" 20 || { echo "[ERROR] $node did not start"; tail -n 40 "$LOGDIR/interface.log"; exit 1; }
done
echo '[BACKEND READY] Start Terminal 3: ~/contest2_uav/demo/06_demo9_v4_success/run_demo10.sh'
echo "[BACKEND] Logs: $LOGDIR"
wait "${PIDS[${#PIDS[@]}-1]}"
