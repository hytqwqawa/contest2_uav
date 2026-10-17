#!/usr/bin/env bash
# Terminal 3: Demo5 V2 mission + advisory Demo7 QR observer. No software ARM.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HOME/contest2_uav"
source "$ROOT/runtime/env.sh"
export ROS_MASTER_URI=http://192.168.5.20:11311
export ROS_IP=192.168.5.7
export ROS_NAMESPACE=/contest2
if rosnode list 2>/dev/null | grep -Fxq /contest2/demo5_qr_monitor; then
    echo '[STOP] Existing Demo7 QR node detected; stop it first.'
    exit 1
fi
BAG_DIR="$ROOT/data/bags/demo5_qr_overlay"
mkdir -p "$BAG_DIR"
BAG="$BAG_DIR/demo7_$(date +%Y%m%d_%H%M%S).bag"
echo "[DEMO7] Flight bag: $BAG"
rosbag record -O "$BAG" \
  /contest2/mavros/state \
  /contest2/mavros/extended_state \
  /contest2/mavros/local_position/odom \
  /contest2/mavros/distance_sensor/hrlv_ez4_pub \
  /contest2/mavros/setpoint_raw/local \
  /contest2/ego_uav_interface/point_traj_status \
  /contest2/ego_uav_interface/trajectory_mux_status \
  /contest2/ego_uav_interface/flight_healthy \
  /contest2/ego_uav_interface/rc_mission_allowed \
  /contest2/ego_uav_interface/demo_frame_safe \
  /contest2/position_cmd \
  /contest2/demo5/qr/result \
  /contest2/demo5/qr/status \
  >/tmp/demo7_qr_rosbag.log 2>&1 &
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
    echo '[ABORT] Demo7 rosbag failed to start'
    tail -n 40 /tmp/demo7_qr_rosbag.log
    exit 1
fi
"$D/start_demo7.sh"
