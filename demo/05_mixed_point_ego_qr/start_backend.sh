#!/usr/bin/env bash

set -u

export ROS_MASTER_URI=http://192.168.8.101:11311
export ROS_IP=192.168.8.102

ROOT="$HOME/contest2_uav"
DEMO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$ROOT/runtime/env.sh"

export ROS_NAMESPACE=/contest2


LOGROOT="$ROOT/data/logs/demo5_qr_overlay"

STAMP="$(date +%Y%m%d_%H%M%S)"

LOGDIR="$LOGROOT/obstacle_hover_$STAMP"

mkdir -p "$LOGDIR"

ln -sfn \
"$LOGDIR" \
"$LOGROOT/obstacle_hover_latest"

D435_PID=""
EXT_PID=""
EGO_PID=""
PROBE_PID=""
IFACE_PID=""

SPEED_CSV="$LOGDIR/speed_compare.csv"
SPEED_SUMMARY="$LOGDIR/speed_compare.summary.txt"
SPEED_LOG="$LOGDIR/speed_probe.log"


wait_topic()
{
    local topic="$1"
    local timeout_s="$2"

    # Keep ONE subscriber alive until the first real message arrives.
    # Repeated 1-s rostopic startup can fail on a remote ROS master
    # even when the topic itself is healthy.
    timeout "${timeout_s}" \
        rostopic echo -n 1 "$topic" \
        >/dev/null 2>&1
}


wait_node()
{
    local node="$1"
    local timeout_s="$2"

    local end=$((SECONDS + timeout_s))

    while [ "$SECONDS" -lt "$end" ]; do

        if rosnode list 2>/dev/null | \
            grep -Fxq "$node"; then

            return 0
        fi

        sleep 0.5
    done

    return 1
}


get_bool()
{
    timeout 5 rostopic echo -n 1 "$1" \
        2>/dev/null | \
        awk '/data:/ {print $2; exit}'
}


cleanup()
{
    trap - EXIT INT TERM

    echo
    echo "========================================"
    echo " STOPPING AUXILIARY EGO STACK"
    echo "========================================"

    if [ -n "$PROBE_PID" ]; then
        kill -INT "$PROBE_PID" 2>/dev/null || true
        sleep 0.5
        wait "$PROBE_PID" 2>/dev/null || true
    fi

    if [ -n "$IFACE_PID" ]; then
        kill -INT "$IFACE_PID" 2>/dev/null || true
        wait "$IFACE_PID" 2>/dev/null || true
    fi

    if [ -n "$EGO_PID" ]; then
        kill -INT "$EGO_PID" 2>/dev/null || true
    fi

    if [ -n "$EXT_PID" ]; then
        kill -INT "$EXT_PID" 2>/dev/null || true
    fi

    if [ -n "$D435_PID" ]; then
        kill -INT "$D435_PID" 2>/dev/null || true
    fi

    sleep 1

    if [ -f "$SPEED_SUMMARY" ]; then

        echo
        cat "$SPEED_SUMMARY"
    fi

    echo
    echo "Logs:"
    echo "$LOGDIR"
}


trap cleanup EXIT
trap 'exit 130' INT TERM


echo
echo "========================================"
echo " CONTEST1 DEMO05 MULTI-WAYPOINT BACKEND"
echo "========================================"


# ---------------------------------------------------------
# Core stack must already be running in terminal 1.
# ---------------------------------------------------------

if ! timeout 3 rostopic list \
    >/dev/null 2>&1; then

    echo "[ABORT] ROS master unavailable."
    exit 1
fi


echo
echo "[0/5] Waiting for terminal-1 core stack..."

echo "      waiting for MAVROS odometry..."

if ! wait_topic \
    /contest2/mavros/local_position/odom \
    60; then

    echo "[ABORT] MAVROS odometry missing after 60 s."
    echo
    echo "Check terminal 1:"
    echo "  ~/contest2_uav/runtime/start_core.sh"
    exit 1
fi

echo "[ OK ] MAVROS odometry available"


wait_bool_true()
{
    local topic="$1"
    local timeout_s="$2"

    local end=$((SECONDS + timeout_s))

    while [ "$SECONDS" -lt "$end" ]; do

        local value

        value="$(get_bool "$topic")"

        if [ "$value" = "True" ]; then
            return 0
        fi

        printf "\r      waiting %-48s current=%-8s" \
            "$topic" "${value:-UNKNOWN}"

        sleep 0.5
    done

    echo
    return 1
}


echo "      waiting for flight_healthy..."

if ! wait_bool_true \
    /contest2/ego_uav_interface/flight_healthy \
    60; then

    echo "[ABORT] flight_healthy did not become True."
    exit 1
fi

echo
echo "[ OK ] flight_healthy = True"


echo "      waiting for offboard stream..."

if ! wait_bool_true \
    /contest2/ego_uav_interface/offboard_stream_ready \
    30; then

    echo "[ABORT] offboard_stream_ready did not become True."
    exit 1
fi

echo
echo "[ OK ] offboard_stream_ready = True"

echo
echo "========================================"
echo " TERMINAL-1 CORE READY"
echo "========================================"


# ---------------------------------------------------------
# Reject duplicated auxiliary stacks.
# ---------------------------------------------------------

for NODE in \
    /contest2/camera/realsense2_camera \
    /contest2/camera_extrinsic_publisher \
    /contest2/drone_0_ego_planner_node \
    /contest2/drone_0_traj_server \
    /contest2/ego_trajectory_bridge \
    /contest2/rviz_goal_executor \
    /contest2/point_traj_node \
    /contest2/trajectory_mux \
    /contest2/uav2_ego_rectangle
do

    if rosnode list 2>/dev/null | \
        grep -Fxq "$NODE"; then

        echo "[ABORT] Duplicate node already running:"
        echo "        $NODE"
        exit 1
    fi
done


# ---------------------------------------------------------
# D435i
# ---------------------------------------------------------

echo
echo "[1/5] Starting D435i..."

roslaunch realsense2_camera rs_camera.launch \
    serial_no:=939722070209 \
    camera:=camera \
    enable_depth:=true \
    enable_color:=true \
    color_width:=640 \
    color_height:=480 \
    color_fps:=30 \
    enable_infra1:=false \
    enable_infra2:=false \
    enable_gyro:=false \
    enable_accel:=false \
    enable_pose:=false \
    depth_width:=640 \
    depth_height:=480 \
    depth_fps:=30 \
    initial_reset:=false \
    diagnostics_topic:=/contest2/diagnostics \
    tf_topic:=/contest2/tf \
    tf_static_topic:=/contest2/tf_static \
    >"$LOGDIR/d435.log" 2>&1 &

D435_PID=$!


if ! wait_topic \
    /contest2/camera/depth/image_rect_raw \
    15; then

    echo "[ABORT] D435 depth did not become ready."
    tail -n 30 "$LOGDIR/d435.log"
    exit 1
fi


echo "[ OK ] D435 depth ready"

echo "      waiting for D435 RGB 640x480@30Hz..."
if ! wait_topic /contest2/camera/color/image_raw 20; then
    echo "[ABORT] D435 RGB unavailable (Demo7 QR needs RGB)."
    tail -n 40 "$LOGDIR/d435.log"
    exit 1
fi
echo "[ OK ] D435 RGB ready; depth remains enabled"



# ---------------------------------------------------------
# Camera extrinsic
# ---------------------------------------------------------

echo
echo "[2/5] Starting camera extrinsic..."

roslaunch ego_uav_interface \
    camera_extrinsic.launch \
    >"$LOGDIR/extrinsic.log" 2>&1 &

EXT_PID=$!


if ! wait_topic \
    /contest2/vins_fusion/extrinsic \
    5; then

    echo "[ABORT] camera extrinsic missing."
    exit 1
fi


echo "[ OK ] camera extrinsic ready"


# ---------------------------------------------------------
# EGO
# ---------------------------------------------------------

echo
echo "[3/5] Starting EGO planner..."

roslaunch ego_uav_interface \
    ego_uav2.launch \
    max_vel:=0.20 \
    >"$LOGDIR/ego.log" 2>&1 &

EGO_PID=$!


if ! wait_node \
    /contest2/drone_0_ego_planner_node \
    12; then

    echo "[ABORT] EGO planner failed to start."
    tail -n 40 "$LOGDIR/ego.log"
    exit 1
fi


if ! wait_node \
    /contest2/drone_0_traj_server \
    5; then

    echo "[ABORT] traj_server failed to start."
    exit 1
fi


if rosnode list | \
    grep -Eiq 'px4ctrl|px4_ctrl'; then

    echo "[ABORT] PX4Ctrl detected."
    exit 1
fi


echo "[ OK ] EGO planner ready"
echo "[ OK ] traj_server ready"
echo "[ OK ] no PX4Ctrl"


echo
echo "EGO runtime velocity parameters:"

rosparam list 2>/dev/null | \
grep '^/contest2/drone_0_ego_planner_node/' | \
grep -E '/(max_vel|limit_vel)$' | \
while read -r P; do

    printf "  %s = " "$P"
    rosparam get "$P" 2>/dev/null || true
done


# ---------------------------------------------------------
# Speed probe
# ---------------------------------------------------------

echo
echo "[4/5] Starting speed diagnostic..."

python3 \
"$DEMO/uav2_speed_probe.py" \
"_csv:=$SPEED_CSV" \
"_summary:=$SPEED_SUMMARY" \
>"$SPEED_LOG" 2>&1 &

PROBE_PID=$!

sleep 1

echo "[ OK ] speed diagnostic active"


# ---------------------------------------------------------
# Mission
# ---------------------------------------------------------

echo
echo "========================================"
echo " BACKEND READY"
echo "========================================"
echo
echo "Next: proven mixed POINT/EGO mission with live QR observer."
echo "The flight program will NOT ARM the aircraft."
echo
echo "Wait for:"
echo
echo "  WAITING FOR PILOT"
echo
echo "Then perform:"
echo
echo "  CH7 : LOW -> MID -> HIGH"
echo "  CH6 : HIGH / POSCTL"
echo "  CH5 : ARM"
echo
echo "After ARM the complete mission starts automatically."
echo
echo "DO NOT press Ctrl-C while airborne."
echo "Use CH7 LOW / CH6 takeover for abnormal flight."
echo
echo "[5/5] Starting mission interface..."
echo


roslaunch \
    "$DEMO/uav2_demo5_multi_waypoint_interface.launch" \
    >"$LOGDIR/point_interface.log" 2>&1 &

IFACE_PID=$!

if ! wait_node /contest2/point_traj_node 8; then
    echo "[ABORT] point_traj_node failed to start."
    tail -n 60 "$LOGDIR/point_interface.log"
    exit 1
fi

if ! wait_node /contest2/trajectory_mux 8; then
    echo "[ABORT] trajectory_mux failed to start."
    tail -n 60 "$LOGDIR/point_interface.log"
    exit 1
fi

if ! wait_node /contest2/ego_trajectory_bridge 8; then
    echo "[ABORT] ego_trajectory_bridge failed to start."
    tail -n 60 "$LOGDIR/point_interface.log"
    exit 1
fi

echo
echo "========================================"
echo " DEMO05 QR BACKEND READY"
echo "========================================"
echo
echo "EGO max_vel = 0.20 m/s"
echo "Demo3 POINT takeoff / landing interface ready; POINT/EGO mixed legs; QR observer runs independently."
echo
echo "Keep this terminal running."
echo
echo "Now start Terminal 3:"
echo "  ~/contest2_uav/demo/05_mixed_point_ego_qr/run_demo7.sh"
echo

wait "$IFACE_PID"

