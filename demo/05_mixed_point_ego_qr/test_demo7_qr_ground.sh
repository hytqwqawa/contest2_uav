#!/usr/bin/env bash
# Ground-only observer test. No goal publishing, no flight services.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HOME/contest2_uav/runtime/env.sh"
export ROS_MASTER_URI=http://192.168.5.20:11311
export ROS_IP=192.168.5.6
export ROS_NAMESPACE=/contest2
STATE="$(timeout -s KILL 5s rostopic echo -n 1 /contest2/mavros/state 2>/dev/null || true)"
printf '%s\n' "$STATE" | grep -Fxq 'armed: False' || { echo '[STOP] PX4 DISARM required'; exit 1; }
printf '%s\n' "$STATE" | grep -Fxq 'mode: "POSCTL"' || { echo '[STOP] POSCTL required'; exit 1; }
if rosnode list 2>/dev/null | grep -Fxq /contest2/demo5_qr_monitor; then
    echo '[STOP] Demo7 QR monitor already running'; exit 1
fi
if ! timeout -s KILL 10s rostopic echo -n 1 /contest2/camera/color/image_raw >/dev/null 2>&1; then
    echo '[STOP] No RGB frames'; exit 1
fi
echo '[DEMO7] Hold QR in view. Recognized payload appears in yellow once per second.'
echo '[DEMO7] Ground test uses advisory-only node, no flight commands.'
exec python3 -u "$D/demo7_qr_monitor.py" _decode_hz:=5.0 _stable_frames:=3 _ground_test:=true
