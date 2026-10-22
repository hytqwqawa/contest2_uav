#!/usr/bin/env bash

set -u

ROOT="$HOME/contest2_uav"
DEMO="$ROOT/demo/01_4m_pole"

source "$ROOT/runtime/env.sh"

export ROS_NAMESPACE=/contest2

export ROS_MASTER_URI=http://192.168.8.102:11311
export ROS_IP=192.168.8.102

BAG_DIR="$ROOT/data/bags/4m_pole"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="$BAG_DIR/final_4m_pole_demo_${STAMP}.bag"

mkdir -p "$BAG_DIR"

echo
echo "=========================================="
echo " UAV2 FINAL 4M POLE DEMO"
echo "=========================================="
echo " POINT takeoff"
echo " EGO obstacle avoidance"
echo " tangent yaw"
echo " 4 m goal"
echo " automatic V4 landing"
echo
echo " NO SOFTWARE ARM"
echo
echo " bag:"
echo " $BAG"
echo "=========================================="
echo

rosbag record \
-O "$BAG" \
/contest2/t265/odom/sample \
/contest2/t265/odom/filtered \
/contest2/mavros/local_position/odom \
/contest2/mavros/state \
/contest2/mavros/extended_state \
/contest2/mavros/distance_sensor/hrlv_ez4_pub \
/contest2/mavros/imu/data \
/contest2/mavros/setpoint_raw/local \
/contest2/ego_uav_interface/point_setpoint \
/contest2/ego_uav_interface/flight_healthy \
/contest2/position_cmd \
/contest2/traj_start_trigger \
/contest2/tf \
/contest2/tf_static \
/rosout_agg \
>/tmp/uav2_final_demo_rosbag.log 2>&1 &

BAG_PID=$!

cleanup()
{
    if kill -0 "$BAG_PID" 2>/dev/null; then
        kill -INT "$BAG_PID" 2>/dev/null
        wait "$BAG_PID" 2>/dev/null
    fi
}

trap cleanup INT TERM EXIT

sleep 1

echo "[ OK ] rosbag recording started"
echo

"$DEMO/start_demo_frame_v2.sh"

RET=$?

cleanup
trap - INT TERM EXIT

echo
echo "=========================================="
echo " FINAL DEMO PROCESS ENDED"
echo " BAG: $BAG"
echo "=========================================="

exit "$RET"
