#!/usr/bin/env bash

set -u

ROOT="$HOME/contest2_uav"
DEMO="$ROOT/demo/02_waypoint_pole"

source "$ROOT/runtime/env.sh"

export ROS_NAMESPACE=/contest2

LOGDIR="$ROOT/data/logs/waypoint_pole/waypoint_pole_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOGDIR"

PIDS=""

add_pid()
{
    PIDS="$PIDS $1"
}

cleanup()
{
    trap - EXIT INT TERM

    echo
    echo "[ CLEANUP ] stopping waypoint-pole backend..."

    for P in $PIDS; do
        kill -INT "$P" 2>/dev/null || true
    done

    sleep 1
}

trap cleanup EXIT INT TERM


wait_topic()
{
    timeout "$2" \
        rostopic echo -n 1 "$1" \
        >/dev/null 2>&1
}


wait_node()
{
    local end=$((SECONDS + $2))

    while [ "$SECONDS" -lt "$end" ]; do

        if rosnode list 2>/dev/null | \
           grep -Fxq "$1"; then
            return 0
        fi

        sleep 0.25
    done

    return 1
}


echo
echo "========================================"
echo " UAV2 WAYPOINT-POLE BACKEND"
echo "========================================"
echo
echo "Required:"
echo "  Terminal 1 core running"
echo "  aircraft at fixed takeoff point"
echo "  DISARMED"
echo "  CH7 LOW"
echo


# ---------------------------------------------------------
# Core ready
# ---------------------------------------------------------

if ! wait_topic \
    /contest2/mavros/local_position/odom \
    30; then

    echo "[ABORT] MAVROS odometry missing"
    exit 1
fi


# ---------------------------------------------------------
# Reject duplicate stacks
# ---------------------------------------------------------

DUP_NODES="
/contest2/camera/realsense2_camera
/contest2/camera_extrinsic_publisher
/contest2/drone_0_ego_planner_node
/contest2/drone_0_traj_server
/contest2/ego_trajectory_bridge
/contest2/rviz_goal_executor
/contest2/waypoint_goal_executor
/contest2/point_traj_node
/contest2/trajectory_mux
/contest2/uav2_demo_frame_guard_v2
/contest2/uav2_waypoint_scene_preview
/contest2/uav2_realtime_path_markers
"

for NODE in $DUP_NODES
do
    if rosnode list 2>/dev/null | grep -Fxq "$NODE"; then
        echo "[ABORT] duplicate node:"
        echo "        $NODE"
        exit 1
    fi
done


# ---------------------------------------------------------
# Demo frame FIRST
# ---------------------------------------------------------

echo "[1/6] Capturing Demo frame..."

PYTHONUNBUFFERED=1 \
python3 \
"$DEMO/uav2_demo_frame_guard_v2.py" \
"_demo_frame_yaml:=$DEMO/demo_frame.yaml" \
"_session_frame_yaml:=$DEMO/session_frame.yaml" \
"_x_min:=-1.50" \
"_x_max:=1.50" \
"_y_min:=-0.50" \
"_y_max:=4.50" \
"_h_min:=-0.25" \
"_h_max:=1.30" \
"_goal_left:=4.00" \
"_flight_height:=0.80" \
>"$LOGDIR/demo_guard.log" 2>&1 &

GUARD_PID=$!
add_pid "$GUARD_PID"

READY=0

for i in $(seq 1 50); do

    V="$(
        timeout 1 \
        rostopic echo -n 1 \
        /contest2/yaw_tangent_demo/session_ready \
        2>/dev/null |
        awk '/data:/ {print $2; exit}'
    )"

    if [ "$V" = "True" ]; then
        READY=1
        break
    fi

    sleep 0.2
done

if [ "$READY" -ne 1 ]; then

    echo "[ABORT] Demo frame failed"
    cat "$LOGDIR/demo_guard.log"
    exit 1
fi

echo "[ OK ] Demo frame ready"


# ---------------------------------------------------------
# D435
# ---------------------------------------------------------

echo "[2/6] Starting D435i..."

roslaunch realsense2_camera rs_camera.launch \
    serial_no:=939722070209 \
    camera:=camera \
    enable_depth:=true \
    enable_color:=false \
    enable_infra1:=false \
    enable_infra2:=false \
    enable_gyro:=false \
    enable_accel:=false \
    enable_pose:=false \
    depth_width:=640 \
    depth_height:=480 \
    depth_fps:=30 \
    initial_reset:=false \
    >"$LOGDIR/d435.log" 2>&1 &

D435_PID=$!
add_pid "$D435_PID"

if ! wait_topic \
    /contest2/camera/depth/image_rect_raw \
    35; then

    echo "[ABORT] D435 depth missing"
    exit 1
fi

echo "[ OK ] D435 ready"


# ---------------------------------------------------------
# Extrinsic
# ---------------------------------------------------------

echo "[3/6] Starting camera extrinsic..."

roslaunch ego_uav_interface \
    camera_extrinsic.launch \
    >"$LOGDIR/extrinsic.log" 2>&1 &

EXT_PID=$!
add_pid "$EXT_PID"

if ! wait_topic \
    /contest2/vins_fusion/extrinsic \
    5; then

    echo "[ABORT] camera extrinsic missing"
    exit 1
fi

echo "[ OK ] camera extrinsic ready"


# ---------------------------------------------------------
# EGO
# ---------------------------------------------------------

echo "[4/6] Starting EGO..."

roslaunch ego_uav_interface \
    ego_uav2.launch \
    >"$LOGDIR/ego.log" 2>&1 &

EGO_PID=$!
add_pid "$EGO_PID"

if ! wait_node \
    /contest2/drone_0_ego_planner_node \
    12; then

    echo "[ABORT] EGO planner missing"
    tail -n 40 "$LOGDIR/ego.log"
    exit 1
fi

if ! wait_node \
    /contest2/drone_0_traj_server \
    5; then

    echo "[ABORT] traj_server missing"
    exit 1
fi

echo "[ OK ] EGO + traj_server ready"


# ---------------------------------------------------------
# POINT / mux / bridge
# ---------------------------------------------------------

echo "[5/6] Starting mission interface..."

roslaunch \
"$DEMO/uav2_waypoint_pole_interface.launch" \
>"$LOGDIR/interface.log" 2>&1 &

IFACE_PID=$!
add_pid "$IFACE_PID"

for NODE in \
    /contest2/point_traj_node \
    /contest2/trajectory_mux \
    /contest2/ego_trajectory_bridge
do

    if ! wait_node "$NODE" 8; then
        echo "[ABORT] missing node $NODE"
        cat "$LOGDIR/interface.log"
        exit 1
    fi
done

echo "[ OK ] POINT / mux / bridge ready"


# ---------------------------------------------------------
# Strict all-leg executor
# ---------------------------------------------------------

echo "[6/6] Starting strict waypoint executor..."

PYTHONUNBUFFERED=1 \
python3 \
"$DEMO/waypoint_goal_executor.py" \
> >(tee "$LOGDIR/executor.log") \
2>&1 &

EXEC_PID=$!
add_pid "$EXEC_PID"

if ! wait_node \
    /contest2/waypoint_goal_executor \
    6; then

    echo "[ABORT] waypoint executor failed"
    cat "$LOGDIR/executor.log"
    exit 1
fi


# ---------------------------------------------------------
# Route markers
# RViz is owned by Terminal 3 / run_mission.sh.
# ---------------------------------------------------------

python3 \
"$DEMO/uav2_waypoint_runtime_markers.py" \
>"$LOGDIR/markers.log" 2>&1 &

MARKER_PID=$!
add_pid "$MARKER_PID"


# ---------------------------------------------------------
# Realtime desired / actual trajectories
# ---------------------------------------------------------

echo "[ OK ] Starting realtime desired / actual paths..."

PYTHONUNBUFFERED=1 python3 "$DEMO/uav2_realtime_path_markers.py" >"$LOGDIR/realtime_paths.log" 2>&1 &

LIVE_PATH_PID=$!
add_pid "$LIVE_PATH_PID"

if ! wait_node     /contest2/uav2_realtime_path_markers     5; then

    echo "[ABORT] realtime path node failed"

    if [ -f "$LOGDIR/realtime_paths.log" ]; then
        cat "$LOGDIR/realtime_paths.log"
    fi

    exit 1
fi

echo "[ OK ] realtime path node ready"




echo "[ OK ] Visualization publishers ready"
echo "       RViz will be started by Terminal 3."


echo
echo "========================================"
echo " WAYPOINT-POLE BACKEND READY"
echo "========================================"
echo
echo "Route:"
echo " P1 = F+1.0 L+1.0"
echo " P2 = F+0.5 L+3.0"
echo " P3 = F-0.5 L+4.0"
echo
echo "Pole:"
echo " F+1.0 L+2.0"
echo
echo "FIRST GAP LIMIT = 0.08 m"
echo
echo "Keep this terminal running."
echo "Terminal 3 owns RViz + rosbag + mission."
echo
echo "Next:"
echo "  ~/contest2_uav/demo/02_waypoint_pole/run_mission.sh"
echo

# Keep the backend alive on the mission-interface roslaunch.
# Closing RViz must never tear down the backend.
wait "$IFACE_PID"
