#!/usr/bin/env bash

set -u

ROOT="$HOME/contest2_uav"
RUNTIME="$ROOT/runtime"
COMMON="$RUNTIME/common"

LAUNCH="$COMMON/launch/uav2_core_bringup.launch"
DASHBOARD="$COMMON/scripts/uav2_system_ready_node.py"

LOG_ROOT="$ROOT/data/logs/core"
ROS_LOG_ROOT="$ROOT/data/logs/ros"

export ROS_MASTER_URI=http://192.168.8.102:11311
export ROS_IP=192.168.8.102

mkdir -p "$LOG_ROOT"
mkdir -p "$ROS_LOG_ROOT"

export ROS_LOG_DIR="$ROS_LOG_ROOT"

# ==========================================================
# Clean canonical runtime environment
# ==========================================================

unset ROS_PACKAGE_PATH
unset CMAKE_PREFIX_PATH

source /opt/ros/noetic/setup.bash
source "$RUNTIME/sunray_ws/devel/setup.bash" --extend
source "$RUNTIME/Fast-Drone-250/devel/setup.bash" --extend
source "$RUNTIME/ego_uav_ws/devel/setup.bash" --extend


# ==========================================================
# Static startup checks
# ==========================================================

echo
echo "============================================================"
echo "              CONTEST2 UAV CORE BRINGUP"
echo "============================================================"
echo
echo "ROS_MASTER_URI : $ROS_MASTER_URI"
echo "ROS_IP         : $ROS_IP"
echo
echo "NO AUTOMATIC ARM"
echo "NO AUTOMATIC OFFBOARD"
echo

printf "%-28s" "[CHECK] ego_uav_interface"
if rospack find ego_uav_interface >/dev/null 2>&1; then
    echo "[ OK ]"
else
    echo "[FAIL]"
    exit 1
fi

printf "%-28s" "[CHECK] sunray_uav_control"
if rospack find sunray_uav_control >/dev/null 2>&1; then
    echo "[ OK ]"
else
    echo "[FAIL]"
    exit 1
fi

printf "%-28s" "[CHECK] realsense2_camera"
if rospack find realsense2_camera >/dev/null 2>&1; then
    echo "[ OK ]"
else
    echo "[FAIL]"
    exit 1
fi

printf "%-28s" "[CHECK] PX4 USB"
if [ -e /dev/ttyACM0 ]; then
    echo "[ OK ]"
else
    echo "[WAIT]"
fi


# ==========================================================
# Core ROS stack
# ==========================================================

STAMP="$(date +%Y%m%d_%H%M%S)"
CORE_LOG="$LOG_ROOT/core_${STAMP}.log"

ln -sfn "$CORE_LOG" \
    "$LOG_ROOT/core_latest.log"

echo
echo "[START] contest2 UAV core nodes"
echo "Log: $CORE_LOG"
echo

roslaunch "$LAUNCH" \
    >"$CORE_LOG" 2>&1 &

CORE_PID=$!


cleanup()
{
    echo
    echo
    echo "Stopping contest2 UAV core stack..."

    if kill -0 "$CORE_PID" 2>/dev/null; then
        kill -INT "$CORE_PID" 2>/dev/null || true
        wait "$CORE_PID" 2>/dev/null || true
    fi

    echo "contest2 UAV core stopped."
}


trap cleanup EXIT INT TERM


sleep 1.0

if ! kill -0 "$CORE_PID" 2>/dev/null; then
    echo
    echo "ERROR: core roslaunch exited during startup."
    echo
    echo "Last log lines:"
    tail -n 80 "$CORE_LOG"
    exit 1
fi


# ==========================================================
# Foreground dashboard
# ==========================================================

ROS_NAMESPACE=/contest2 python3 -u "$DASHBOARD"
