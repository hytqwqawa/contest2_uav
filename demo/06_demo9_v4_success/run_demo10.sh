#!/usr/bin/env bash
# Terminal 3: single-level mission launcher. CH7 + physical ARM are unchanged.
set -eo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HOME/contest2_uav"
source "$ROOT/runtime/env.sh"
export DEMO10_QR_AFTER_WAYPOINT=1
export ROS_MASTER_URI=http://192.168.8.102:11311
export ROS_IP=192.168.8.102
export ROS_NAMESPACE=/contest2
python3 -B "$D/demo5_config.py" --check "$D/demo5_waypoints.yaml"
LOGDIR="$ROOT/data/logs/demo10_compact/mission_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOGDIR"
if rosnode list 2>/dev/null | grep -Eq '^/contest2/(uav2_demo5_waypoints|contest2_demo_frame_guard_v2)$'; then
  echo '[ERROR] previous mission/frame guard is still running'; exit 1
fi
if ! rosnode list 2>/dev/null | grep -Fxq /contest2/ego_trajectory_bridge; then
  echo '[ERROR] backend not ready; start Terminal 2 first'; exit 1
fi
PIDS=()
armed_now() {
  local data
  data="$(timeout 3 rostopic echo -n 1 /contest2/mavros/state 2>/dev/null || true)"
  [[ "$(printf '%s\n' "$data" | awk '/^armed:/ {print $2; exit}')" == 'True' ]]
}
cleanup() {
  trap - EXIT INT TERM
  if armed_now; then
    echo '[PILOT] Flight still ARMED. CH7 LOW + CH6 POSCTL takeover; land and physical DISARM.'
    echo '[PILOT] Frame guard remains alive until the vehicle disarms.'
    while armed_now; do sleep 1; done
  fi
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do kill -INT "${PIDS[$i]}" 2>/dev/null || true; done
  for pid in "${PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM
rm -f "$D/session_frame.yaml" "$D/alignment_target.yaml"
echo '[MISSION] Capturing DISARMED takeoff frame...'
PYTHONUNBUFFERED=1 python3 -u "$D/uav2_demo_frame_guard_v2.py" >"$LOGDIR/frame.log" 2>&1 & PIDS+=("$!")
ready=0
for ((n=0; n<45; n++)); do
  if ! kill -0 "${PIDS[0]}" 2>/dev/null; then break; fi
  state="$(timeout 2 rostopic echo -n 1 /contest2/yaw_tangent_demo/session_ready 2>/dev/null | awk '/data:/ {print $2; exit}' || true)"
  if [ "$state" = 'True' ] && [ -s "$D/session_frame.yaml" ]; then ready=1; break; fi
  sleep 0.3
done
if [ "$ready" -ne 1 ]; then
  echo '[ERROR] takeoff frame not established'; tail -n 45 "$LOGDIR/frame.log"; exit 1
fi
echo '[MISSION] Frame ready. Inspect numbered waypoints/obstacles before CH7 and ARM.'
# Optional visualization; failure must not veto the flight setup itself.
if [ "${SHOW_RVIZ:-1}" = '1' ] && command -v rviz >/dev/null 2>&1; then
  rviz -d "$D/yaw_tangent_demo.rviz" /tf:=/contest2/tf /tf_static:=/contest2/tf_static \
    >"$LOGDIR/rviz.log" 2>&1 & PIDS+=("$!")
fi

if command -v rosbag >/dev/null 2>&1; then
  mkdir -p "$ROOT/data/bags/demo10_compact"
  rosbag record -O "$ROOT/data/bags/demo10_compact/mission_$(date +%Y%m%d_%H%M%S).bag" \
    /contest2/mavros/state /contest2/mavros/local_position/odom \
    /contest2/mavros/setpoint_raw/local /contest2/mavros/rc/in \
    /contest2/ego_uav_interface/flight_healthy \
    /contest2/ego_uav_interface/rc_mission_allowed \
    /contest2/ego_uav_interface/trajectory_mux_status \
    /contest2/ego_uav_interface/trajectory_interface_status \
    /contest2/ego_uav_interface/point_traj_status \
    /contest2/ego_uav_interface/point_traj_result \
    /contest2/demo5/active_waypoint /contest2/position_cmd \
    >"$LOGDIR/rosbag.log" 2>&1 & PIDS+=("$!")
fi
echo '[MISSION] Waiting for CH7 authorization and physical ARM (no software ARM).' 
python3 -B -u "$D/pilot_takeoff.py"
echo '[MISSION] Takeoff accepted; running configured EGO/POINT waypoints.'
python3 -B -u "$D/uav2_demo5_waypoints.py" 2>&1 | tee "$LOGDIR/waypoints.log"
echo '[MISSION] Waypoint runner completed.'
