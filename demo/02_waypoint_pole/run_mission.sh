#!/usr/bin/env bash

set -u

ROOT="$HOME/contest2_uav"
DEMO="$ROOT/demo/02_waypoint_pole"

source "$ROOT/runtime/env.sh"

export ROS_NAMESPACE=/contest2

BAGDIR="$ROOT/data/bags/waypoint_pole"
STAMP="$(date +%Y%m%d_%H%M%S)"

mkdir -p "$BAGDIR"

BAG="$BAGDIR/waypoint_pole_demo_${STAMP}.bag"

echo
echo "========================================"
echo " UAV2 WAYPOINT-POLE FULL DEMO"
echo "========================================"
echo
echo " P0 -> P1 -> P2(obstacle) -> P3 -> LAND"
echo
echo "NO SOFTWARE ARM"
echo
echo "bag:"
echo "$BAG"
echo


rosbag record \
-O "$BAG" \
/contest2/t265/odom/filtered \
/contest2/mavros/local_position/odom \
/contest2/mavros/state \
/contest2/mavros/extended_state \
/contest2/mavros/distance_sensor/hrlv_ez4_pub \
/contest2/mavros/setpoint_raw/local \
/contest2/ego_uav_interface/point_setpoint \
/contest2/ego_uav_interface/ego_setpoint \
/contest2/ego_uav_interface/trajectory_mux_status \
/contest2/ego_uav_interface/rviz_goal_status \
/contest2/ego_uav_interface/demo_frame_safe \
/contest2/ego_uav_interface/yaw_mode \
/contest2/ego_uav_interface/yaw_target_deg \
/contest2/drone_0_planning/bspline \
/contest2/position_cmd \
/contest2/traj_start_trigger \
/rosout_agg \
>/tmp/waypoint_pole_bag.log 2>&1 &

BAG_PID=$!
RVIZ_PID=""

cleanup()
{
    trap - EXIT INT TERM

    if [ -n "$RVIZ_PID" ] && \
       kill -0 "$RVIZ_PID" 2>/dev/null; then
        kill -INT "$RVIZ_PID" 2>/dev/null || true
        wait "$RVIZ_PID" 2>/dev/null || true
    fi

    if kill -0 "$BAG_PID" 2>/dev/null; then
        kill -INT "$BAG_PID" 2>/dev/null || true
        wait "$BAG_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

sleep 1

# Formal Demo02 owns exactly one contest2 RViz instance here.
# Refuse to stack another preview/formal RViz on top.
if rosnode list 2>/dev/null | \
   grep -Eq '^/contest2/(rviz|waypoint_pole_rviz)$'; then

    echo "[ABORT] contest2 RViz already running."
    echo "        Close the old RViz / scene preview first."
    exit 1
fi

echo "[ RVIZ ] Starting formal waypoint-pole view..."

rviz \
    -d "$DEMO/waypoint_pole_demo.rviz" \
    __name:=waypoint_pole_rviz \
    /tf:=/contest2/tf \
    /tf_static:=/contest2/tf_static &

RVIZ_PID=$!

sleep 2

if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "[ABORT] RViz failed to start."
    exit 1
fi

echo "[ OK ] RViz ready"

echo
echo "[ LIVE PATH ] resetting previous mission paths"

rostopic pub -1 /contest2/waypoint_pole/reset_live_paths std_msgs/Empty "{}" >/dev/null 2>&1 || true

sleep 0.2

PYTHONUNBUFFERED=1 \
python3 \
"$DEMO/uav2_waypoint_pole_mission.py"

RET=$?

cleanup
trap - EXIT INT TERM

echo
echo "========================================"
echo " TEST FINISHED"
echo " BAG:"
echo " $BAG"
echo "========================================"

exit "$RET"
