#!/usr/bin/env bash
DEMO10_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -B "$DEMO10_SCRIPT_DIR/demo10_verify.py" --quiet || {
  echo "[STOP] Demo10 source integrity check failed"
  exit 96
}
# Terminal 3: Demo9 integrated EGO yaw mission + advisory Demo7 QR observer. No software ARM.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HOME/contest2_uav"
source "$ROOT/runtime/env.sh"
V4_MUX="/home/yundrone/contest2_uav/demo/10_demo9_v4_success/ros_src/ego_uav_interface/scripts/trajectory_mux_v4_only.py"
if ! pgrep -fa "$V4_MUX" >/dev/null 2>&1; then
  echo "[STOP] Running backend is not V4 MUX; mission NOT started"
  exit 96
fi
echo "[V4 VERIFIED] Live MUX process uses $V4_MUX"
python3 -B "$D/demo10_verify.py" --mux-only
export ROS_MASTER_URI=http://192.168.8.101:11311
export ROS_IP=192.168.8.102
export ROS_NAMESPACE=/contest2
if rosnode list 2>/dev/null | grep -Fxq /contest2/demo7_qr_monitor; then
    echo '[STOP] Existing Demo7 QR node detected; stop it first.'
    exit 1
fi
BAG_DIR="$ROOT/data/bags/demo10_v4_success"
mkdir -p "$BAG_DIR"
BAG="$BAG_DIR/demo10_$(date +%Y%m%d_%H%M%S).bag"
echo "[DEMO9] Flight bag: $BAG"
rosbag record -O "$BAG" \
  /contest2/mavros/state \
  /contest2/mavros/extended_state \
  /contest2/mavros/local_position/odom \
  /contest2/mavros/distance_sensor/hrlv_ez4_pub \
  /contest2/mavros/setpoint_raw/local \
  /contest2/mavros/rc/in \
  /contest2/mavros/battery \
  /contest2/mavros/imu/data \
  /contest2/mavros/estimator_status \
  /contest2/t265/odom/filtered \
  /contest2/ego_uav_interface/point_setpoint \
  /contest2/ego_uav_interface/trajectory_setpoint \
  /contest2/demo9_integrated/mission_event \
  /contest2/ego_uav_interface/point_traj_status \
  /contest2/ego_uav_interface/trajectory_mux_status \
  /contest2/ego_uav_interface/flight_healthy \
  /contest2/ego_uav_interface/rc_mission_allowed \
  /contest2/ego_uav_interface/demo_frame_safe \
  /contest2/position_cmd \
  /contest2/demo7/qr/result \
  /contest2/demo7/qr/status \
  /contest2/ego_uav_interface/yaw_mode \
  /contest2/ego_uav_interface/yaw_target_deg \
  /contest2/ego_uav_interface/ego_bridge_status \
  /contest2/ego_uav_interface/trajectory_interface_status \
  /contest2/ego_uav_interface/ego_setpoint \
  /contest2/demo5/active_waypoint \
  >/tmp/demo10_integrated_yaw_rosbag.log 2>&1 &
BAG_PID=$!
cleanup() {
    trap - EXIT INT TERM
    if kill -0 "$BAG_PID" 2>/dev/null; then
        kill -INT "$BAG_PID" 2>/dev/null || true
        wait "$BAG_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
sleep 1
if ! kill -0 "$BAG_PID" 2>/dev/null; then
    echo '[ABORT] Demo9 rosbag failed to start'
    tail -n 40 /tmp/demo10_integrated_yaw_rosbag.log
    exit 1
fi
"$D/start_demo9.sh"
