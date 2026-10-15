#!/usr/bin/env bash

set -u

ROOT="$HOME/contest2_uav"
DBG="$ROOT/demo/04_5m_pole"

source "$ROOT/runtime/env.sh"

export ROS_NAMESPACE=/contest2

rm -f "$DBG/alignment_target.yaml"

GUARD_PID=""
PREVIEW_PID=""
RVIZ_PID=""

cleanup()
{
    trap - EXIT INT TERM

    echo
    echo "[ CLEANUP ] stopping Demo04 auxiliary nodes..."

    [ -n "${PREVIEW_PID:-}" ] && \
        kill -INT "$PREVIEW_PID" 2>/dev/null || true

    [ -n "${RVIZ_PID:-}" ] && \
        kill -INT "$RVIZ_PID" 2>/dev/null || true

    [ -n "${GUARD_PID:-}" ] && \
        kill -INT "$GUARD_PID" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

echo
echo "========================================"
echo " CONTEST2 DEMO04 - 5M POLE"
echo " Demo1 tangent/EGO + Demo3 takeoff/LAND"
echo "========================================"
echo
echo "Required at startup:"
echo "  aircraft at fixed takeoff point"
echo "  CH5 DISARM"
echo "  CH6 POSCTL"
echo "  CH7 LOW"
echo "  CH8 SAFE"
echo

for F in \
"$DBG/uav2_demo_frame_guard_v2.py" \
"$DBG/uav2_tangent_preview.py" \
"$DBG/uav2_5m_takeoff_gate.py" \
"$DBG/uav2_flight_ops_dev.py" \
"$DBG/demo_frame.yaml" \
"$DBG/yaw_tangent_demo.rviz"
do
    if [ ! -f "$F" ]; then
        echo "[ABORT] missing:"
        echo "        $F"
        exec /bin/false
    fi
done

chmod +x \
"$DBG/uav2_demo_frame_guard_v2.py" \
"$DBG/uav2_tangent_preview.py" \
"$DBG/uav2_5m_takeoff_gate.py" \
"$DBG/uav2_flight_ops_dev.py"

rm -f \
"$DBG/demo_frame_guard_v2.log" \
"$DBG/tangent_preview.log" \
"$DBG/preview_summary.txt" \
"$DBG/session_frame.yaml"

echo "[1/4] Capturing 5m Demo frame..."

PYTHONUNBUFFERED=1 \
python3 \
"$DBG/uav2_demo_frame_guard_v2.py" \
>"$DBG/demo_frame_guard_v2.log" \
2>&1 &

GUARD_PID=$!

READY=0

for i in $(seq 1 50); do

    if ! kill -0 "$GUARD_PID" 2>/dev/null; then
        echo
        echo "[ABORT] Demo-frame guard exited:"
        cat "$DBG/demo_frame_guard_v2.log"
        exec /bin/false
    fi

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
    echo
    echo "[ABORT] Demo-frame session not ready."
    cat "$DBG/demo_frame_guard_v2.log"
    exec /bin/false
fi

echo "[ OK ] Demo-frame session ready"
echo
cat "$DBG/session_frame.yaml"
echo

echo "[2/4] Starting preset RViz..."

rviz \
-d "$DBG/yaw_tangent_demo.rviz" \
/tf:=/contest2/tf \
/tf_static:=/contest2/tf_static &

RVIZ_PID=$!

sleep 1

if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "[ABORT] RViz exited immediately."
    exec /bin/false
fi

echo
echo "========================================"
echo " CHECK RVIZ BEFORE ARM"
echo "========================================"
echo "Goal must show:"
echo "  Demo forward = 0.00 m"
echo "  Demo left    = 5.00 m"
echo "  height       = 0.80 m"
echo
echo "Fence longitudinal left_max = 5.50 m"
echo

echo "[3/4] Waiting for pilot + Demo3 takeoff..."

PYTHONUNBUFFERED=1 \
python3 \
"$DBG/uav2_5m_takeoff_gate.py"

TAKEOFF_RET=$?

if [ "$TAKEOFF_RET" -ne 0 ]; then
    echo
    echo "[ABORT] Demo04 takeoff gate failed."
    exec /bin/false
fi

echo
echo "[ OK ] Demo3 takeoff physically settled"
echo

echo "[4/4] Starting Demo1 5m tangent/EGO mission..."

PYTHONUNBUFFERED=1 \
python3 \
"$DBG/uav2_tangent_preview.py" \
> >(tee "$DBG/tangent_preview.log") \
2>&1 &

PREVIEW_PID=$!

sleep 1

if ! kill -0 "$PREVIEW_PID" 2>/dev/null; then
    echo
    echo "[ABORT] tangent mission exited immediately:"
    cat "$DBG/tangent_preview.log"
    exec /bin/false
fi

MISSION_RET=0

wait "$PREVIEW_PID" || MISSION_RET=$?

if [ "$MISSION_RET" -ne 0 ]; then
    echo
    echo "[FAIL] Demo04 tangent mission returned $MISSION_RET"
    tail -n 100 "$DBG/tangent_preview.log"
    exec /bin/false
fi

echo
echo "========================================"
echo " DEMO04 5M MISSION PROCESS COMPLETE"
echo "========================================"
echo
