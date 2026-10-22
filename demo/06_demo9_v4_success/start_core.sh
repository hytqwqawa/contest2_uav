#!/usr/bin/env bash
# Terminal 1. Keep the original ROS/PX4 core; no automatic ARM or OFFBOARD.
set -eo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HOME/contest2_uav"
RUNTIME="$ROOT/runtime"
LOGROOT="$ROOT/data/logs/core_06_demo9_v4_success"
mkdir -p "$LOGROOT" "$ROOT/data/logs/ros"
export ROS_MASTER_URI=http://192.168.8.102:11311
export ROS_IP=192.168.8.102
export ROS_LOG_DIR="$ROOT/data/logs/ros"
unset ROS_PACKAGE_PATH CMAKE_PREFIX_PATH
source /opt/ros/noetic/setup.bash
source "$RUNTIME/sunray_ws/devel/setup.bash" --extend
source "$RUNTIME/Fast-Drone-250/devel/setup.bash" --extend
source "$RUNTIME/ego_uav_ws/devel/setup.bash" --extend
# ROS requires a package directory for the interface overlay.
export ROS_PACKAGE_PATH="$D:${ROS_PACKAGE_PATH:-}"
for pkg in ego_uav_interface sunray_uav_control realsense2_camera demo8_interface_overlay; do
  rospack find "$pkg" >/dev/null || { echo "[ERROR] missing ROS package: $pkg"; exit 1; }
done
STAMP="$(date +%Y%m%d_%H%M%S)"
SAN_LOG="$LOGROOT/sanitizer_$STAMP.log"
CORE_LOG="$LOGROOT/core_$STAMP.log"
ROS_NAMESPACE=/contest2 python3 -u "$D/t265_landing_sanitizer_dev.py" >"$SAN_LOG" 2>&1 &
SAN_PID=$!
roslaunch "$D/uav2_core_bringup_dev.launch" >"$CORE_LOG" 2>&1 &
CORE_PID=$!
cleanup() {
  trap - EXIT INT TERM
  kill -INT "$SAN_PID" "$CORE_PID" 2>/dev/null || true
  wait "$SAN_PID" "$CORE_PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
sleep 1
if ! kill -0 "$SAN_PID" 2>/dev/null || ! kill -0 "$CORE_PID" 2>/dev/null; then
  echo '[ERROR] core start failed'; tail -n 35 "$SAN_LOG" "$CORE_LOG"; exit 1
fi
echo "[CORE] ROS core processes running (no software ARM)."
echo "[CORE] logs: $CORE_LOG | $SAN_LOG"
# Preserves the familiar readiness display for Terminal 1.
ROS_NAMESPACE=/contest2 python3 -u "$RUNTIME/common/scripts/uav2_system_ready_node.py"
