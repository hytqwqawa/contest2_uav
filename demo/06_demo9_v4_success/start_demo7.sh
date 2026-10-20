#!/usr/bin/env bash
# Thin QR observer wrapper; calls UNMODIFIED successful Demo5 V2 pilot+flight mission.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HOME/contest2_uav"
source "$ROOT/runtime/env.sh"
export ROS_MASTER_URI=http://192.168.8.101:11311
export ROS_IP=192.168.8.102
export ROS_NAMESPACE=/contest2

if rosnode list 2>/dev/null | grep -Fxq /contest2/demo7_qr_monitor; then
    echo '[ABORT] Demo7 QR monitor already running; stop duplicate.'
    exit 1
fi
# Fail BEFORE pilot gate if the RGB pipeline is not ready.
if ! timeout -s KILL 10s rostopic echo -n 1 \
    /contest2/camera/color/image_raw >/dev/null 2>&1; then
    echo '[ABORT] D435i RGB topic has no frames.'
    exit 1
fi

echo '[DEMO7] Starting non-controlling QR observer (yellow 1Hz during ARMED/OFFBOARD).'
PYTHONUNBUFFERED=1 python3 -u "$D/demo7_qr_monitor.py" \
    _decode_hz:=5.0 _stable_frames:=3 _ground_test:=false &
QR_PID=$!
cleanup() {
    trap - EXIT INT TERM
    if kill -0 "$QR_PID" 2>/dev/null; then
        kill -INT "$QR_PID" 2>/dev/null || true
        wait "$QR_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

sleep 2
if ! kill -0 "$QR_PID" 2>/dev/null; then
    echo '[ABORT] Demo7 QR monitor failed before pilot gate.'
    exit 1
fi
if ! timeout -s KILL 6s rostopic echo -n 1 \
    /contest2/demo7/qr/status >/dev/null 2>&1; then
    echo '[ABORT] Demo7 QR node not publishing status.'
    exit 1
fi

echo '[ OK ] QR monitor online; flight mission unchanged.'
echo '[DEMO7] QR observations DO NOT alter POINT/EGO/landing or pilot permissions.'
"$D/start_demo5.sh"
