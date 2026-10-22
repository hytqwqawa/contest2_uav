#!/usr/bin/env bash
# Ground-only performance check with real 5Hz QR decode; does NOT issue flight commands.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HOME/contest2_uav/runtime/env.sh"
export ROS_MASTER_URI=http://192.168.8.102:11311
export ROS_IP=192.168.8.102
export ROS_NAMESPACE=/contest2
STATE="$(timeout -s KILL 5s rostopic echo -n 1 /contest2/mavros/state 2>/dev/null || true)"
printf '%s\n' "$STATE" | grep -Fxq 'armed: False' || { echo '[STOP] PX4 DISARM required'; exit 1; }
printf '%s\n' "$STATE" | grep -Fxq 'mode: "POSCTL"' || { echo '[STOP] POSCTL required'; exit 1; }
if rosnode list 2>/dev/null | grep -Fxq /contest2/demo5_qr_monitor; then
    echo '[STOP] Stop existing QR test first'; exit 1
fi
LOG="/tmp/demo7_camera_load_$(date +%Y%m%d_%H%M%S).log"
PYTHONUNBUFFERED=1 python3 -u "$D/demo7_qr_monitor.py" \
    _decode_hz:=5.0 _stable_frames:=3 _ground_test:=true >"$LOG" 2>&1 &
PID=$!
cleanup() {
    trap - EXIT INT TERM
    kill -INT "$PID" 2>/dev/null || true
    wait "$PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
sleep 2
if ! kill -0 "$PID" 2>/dev/null; then
    echo '[FAIL] QR monitor exited'; cat "$LOG"; exit 1
fi
echo '[DEMO7] Keep QR visible for the ENTIRE frequency test.'
for TOPIC in \
    /contest2/camera/depth/image_rect_raw \
    /contest2/camera/color/image_raw \
    /contest2/mavros/local_position/odom
do
    F="$(mktemp /tmp/demo7_rosrate_XXXXXX)"
    echo "=== $TOPIC ==="
    timeout -s INT -k 3s 12s env PYTHONUNBUFFERED=1 rostopic hz "$TOPIC" >"$F" 2>&1 || true
    grep 'average rate:' "$F" | tail -n 1 || true
    RATE="$(awk '/average rate:/{x=$3} END{print x+0}' "$F")"
    rm -f "$F"
    if ! awk -v r="$RATE" 'BEGIN{exit !(r>=25.0)}'; then
        echo "[FAIL] Topic frequency <25Hz: $TOPIC"
        echo "QR log: $LOG"
        exit 1
    fi
done
if ! grep -Fq '[DEMO7 QR] 1Hz detected:' "$LOG"; then
    echo '[FAIL] No real QR detected during concurrent throughput test.'
    echo 'Hold an actual QR in the camera view and repeat.'
    echo "QR log: $LOG"
    exit 1
fi
echo '[ OK ] QR detection + RGB/Depth/ODOM >=25Hz concurrently'
echo 'QR samples (yellow when viewed in a terminal):'
grep -F '[DEMO7 QR] 1Hz detected:' "$LOG" | tail -n 4 || true
echo "QR log: $LOG"
