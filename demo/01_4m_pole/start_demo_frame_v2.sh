#!/usr/bin/env bash

set -u

ROOT="$HOME/contest2_uav"
DBG="$ROOT/demo/01_4m_pole"

source "$ROOT/runtime/env.sh"

export ROS_NAMESPACE=/contest2

# Remove stale ALIGN target from previous flight
rm -f "$DBG/alignment_target.yaml"

GUARD_PID=""
PREVIEW_PID=""
RVIZ_PID=""

cleanup()
{
    trap - EXIT INT TERM

    echo
    echo "[ CLEANUP ] stopping Demo-frame auxiliary nodes..."

    [ -n "${RVIZ_PID:-}" ] && \
        kill -INT "$RVIZ_PID" 2>/dev/null || true

    [ -n "${PREVIEW_PID:-}" ] && \
        kill -INT "$PREVIEW_PID" 2>/dev/null || true

    [ -n "${GUARD_PID:-}" ] && \
        kill -INT "$GUARD_PID" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

echo
echo "========================================"
echo " UAV2 YAW-TANGENT DEMO"
echo " PLAN -> ALIGN -> EXECUTE architecture"
echo "========================================"
echo
echo "Current stage:"
echo "  PLAN ONLY"
echo "  aircraft remains POINT HOLD"
echo
echo "Required at startup:"
echo "  fixed takeoff point"
echo "  DISARMED"
echo "  CH7 LOW"
echo

# ----------------------------------------------------------
# Static file checks
# ----------------------------------------------------------

for F in \
"$DBG/uav2_demo_frame_guard_v2.py" \
"$DBG/uav2_tangent_preview.py" \
"$DBG/demo_frame.yaml" \
"$DBG/yaw_tangent_demo.rviz"
do

    if [ ! -f "$F" ]; then
        echo "[ABORT] missing:"
        echo "        $F"
        exit 1
    fi

done

chmod +x \
"$DBG/uav2_demo_frame_guard_v2.py" \
"$DBG/uav2_tangent_preview.py"

# ----------------------------------------------------------
# Start Demo-frame guard
# ----------------------------------------------------------

rm -f \
"$DBG/demo_frame_guard_v2.log" \
"$DBG/tangent_preview.log" \
"$DBG/preview_summary.txt"

echo "[1/3] Starting Demo-frame guard..."

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
        exit 1
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
    exit 1
fi

echo "[ OK ] Demo-frame session ready"

echo
cat "$DBG/session_frame.yaml"

# ----------------------------------------------------------
# Start preview process
#
# IMPORTANT:
# output goes BOTH to screen and tangent_preview.log
# ----------------------------------------------------------

echo
echo "[2/3] Starting EGO tangent preview watcher..."

PYTHONUNBUFFERED=1 \
python3 \
"$DBG/uav2_tangent_preview.py" \
> >(tee "$DBG/tangent_preview.log") \
2>&1 &

PREVIEW_PID=$!

sleep 1

if ! kill -0 "$PREVIEW_PID" 2>/dev/null; then

    echo
    echo "[ABORT] tangent preview exited immediately:"
    cat "$DBG/tangent_preview.log"
    exit 1
fi

echo
echo "[ OK ] tangent preview process is running"
echo "       PID=$PREVIEW_PID"
echo "       log=$DBG/tangent_preview.log"
echo
echo "It should now print WAIT status until"
echo "POINT reaches AIRBORNE_HOLD."
echo

# ----------------------------------------------------------
# RViz
# ----------------------------------------------------------

echo "[3/3] Starting preset RViz..."

rviz \
-d "$DBG/yaw_tangent_demo.rviz" \
/tf:=/contest2/tf \
/tf_static:=/contest2/tf_static &

RVIZ_PID=$!

echo
echo "========================================"
echo " TERMINAL 2 READY"
echo "========================================"
echo
echo "Now start the POINT hover mission"
echo "in TERMINAL 3."
echo
echo "DO NOT close this terminal."
echo

wait "$RVIZ_PID"
