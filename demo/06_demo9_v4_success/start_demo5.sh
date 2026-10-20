#!/usr/bin/env bash
DEMO10_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -B "$DEMO10_SCRIPT_DIR/demo10_verify.py" --quiet || {
  echo "[STOP] Demo10 source integrity check failed"
  exit 96
}

set -u

ROOT="$HOME/contest2_uav"
DBG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$ROOT/runtime/env.sh"

export ROS_NAMESPACE=/contest2

# Pure Python config validation; never ARM or launch ROS from here.
if ! python3 "$DBG/demo5_config.py" --check "$DBG/demo5_waypoints.yaml"; then
    echo "[ABORT] Invalid Demo5 waypoints/fence YAML."
    exit 1
fi


# DEMO10_FOUR_LEG_GUARD: verify exact four-point plan before pilot gate.
python3 -B - "$DBG/demo5_waypoints.yaml" <<'PYV4FOUR'
import sys, yaml
p = yaml.safe_load(open(sys.argv[1]))
expected = [
  {'forward':3.0,'left':0.0,'mode':'EGO','yaw_deg':0.0},
  {'forward':3.0,'left':-1.0,'mode':'EGO','yaw_deg':-90.0},
  {'forward':2.0,'left':-1.5,'mode':'EGO','yaw_deg':-180.0},
  {'forward':1.0,'left':-1.5,'mode':'POINT'},
]
if p.get('waypoints') != expected or p.get('flight_height') != 0.8:
    raise SystemExit('[STOP] Not the reviewed four-waypoint Demo9 plan')
print('[V4 VERIFIED] FOUR LEGS: EGO, EGO, EGO, POINT')
PYV4FOUR

echo "[CHECK] DISARMED aircraft nose must face NEW forward (old LEFT)."
if rosnode list 2>/dev/null | grep -Eq '^/contest2/(contest2_demo_frame_guard_v2|uav2_demo_frame_guard_v2|uav2_demo5_waypoints|uav2_tangent_preview)$'; then
    echo "[ABORT] Existing Demo-frame or waypoint mission node detected."
    exit 1
fi
rm -f "$DBG/alignment_target.yaml"

GUARD_PID=""
PREVIEW_PID=""
RVIZ_PID=""

cleanup()
{
    trap - EXIT INT TERM

    echo
    echo "[ CLEANUP ] stopping Demo9 auxiliary nodes..."

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
echo " contest2 DEMO9 V2 - POINT/EGO WAYPOINTS"
echo " YAML POINT/EGO legs + V3 POINT takeoff + POSCTL pilot landing"
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
"$DBG/demo5_config.py" \
"$DBG/demo5_waypoints.yaml" \
"$DBG/uav2_demo5_waypoints.py" \
"$DBG/uav2_5m_takeoff_gate.py" \
"$DBG/uav2_flight_ops_dev.py" \
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
"$DBG/uav2_demo5_waypoints.py" \
"$DBG/uav2_5m_takeoff_gate.py" \
"$DBG/uav2_flight_ops_dev.py"

rm -f \
"$DBG/demo_frame_guard_v2.log" \
"$DBG/demo5_waypoints.log" \
"$DBG/preview_summary.txt" \
"$DBG/session_frame.yaml"

echo "[1/4] Capturing new-nose Demo frame..."

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
if [ ! -s "$DBG/session_frame.yaml" ]; then
    echo "[ABORT] Frame Guard did not create DEMO10 session_frame.yaml"
    exit 1
fi

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
    exit 1
fi

echo
echo "========================================"
echo " CHECK ALL WAYPOINTS IN RVIZ BEFORE ARM"
echo "========================================"
echo "Demo5: NOSE MUST POINT NEW FORWARD (= Demo4 old LEFT)."
echo "Fence: F [-1.00,+6.00] L [-2.00,+3.00]"
echo "Check EVERY numbered waypoint marker against field and obstacles."
echo "The last waypoint is the POSCTL pilot-landing handoff location."
echo
echo "[INFO] Visually verify RViz before CH7 permission / CH5 ARM."
echo "[INFO] Pilot gate still requires physical authorization; no software ARM."

# Refuse changed waypoint settings after Guard/RViz frame capture.
# Compare normalized, validated flight parameters; no SHA or source fingerprints.
if ! python3 -B - "$DBG" <<'PYPLAN'
import sys, yaml
from pathlib import Path
root = Path(sys.argv[1])
sys.path.insert(0, str(root))
from demo5_config import load_config
session = yaml.safe_load((root / 'session_frame.yaml').read_text())
if session.get('demo5_plan_snapshot') != load_config(root / 'demo5_waypoints.yaml'):
    raise SystemExit('[ABORT] Demo5 YAML changed after RViz capture')
print('[ OK ] RViz/Guard waypoint configuration matches')
PYPLAN
then
    exit 1
fi

echo "[3/4] Waiting for pilot + Demo3 takeoff..."

PYTHONUNBUFFERED=1 \
python3 \
"$DBG/uav2_5m_takeoff_gate.py"

TAKEOFF_RET=$?

# Keep Guard + RViz alive on abnormal airborne abort until pilot takeover.
wait_safe_pilot_takeover()
{
    echo
    echo "[PILOT] If airborne: CH7 LOW, CH6 POSCTL; do not software DISARM."
    echo "[PILOT] Leave this terminal running until aircraft is safely handled."
    while true; do
        RAW="$(timeout 3 rostopic echo -n 1 /contest2/mavros/state 2>/dev/null || true)"
        ARMED="$(printf '%s\n' "$RAW" | awk '/^armed:/ {print $2; exit}')"
        MODE="$(printf '%s\n' "$RAW" | awk '/^mode:/ {print $2; exit}')"
        if [ "$ARMED" = "False" ]; then
            echo "[ OK ] PX4 DISARMED; auxiliary nodes may stop."
            return 0
        fi
        if [ "$ARMED" = "True" ] && [ "$MODE" = "POSCTL" ]; then
            echo "[PILOT] POSCTL takeover active; waiting for physical landing/DISARM."
        fi
        echo "[WAIT] armed=${ARMED:-UNKNOWN} mode=${MODE:-UNKNOWN} | CH7 LOW + CH6 POSCTL"
        sleep 1
    done
}

if [ "$TAKEOFF_RET" -ne 0 ]; then
    echo
    echo "[ABORT] Demo9 takeoff gate failed."
    wait_safe_pilot_takeover
    exit 1
fi

echo
echo "[ OK ] V3 takeoff physically settled"
echo

echo "[4/4] Starting mixed POINT/EGO waypoint mission..."

PYTHONUNBUFFERED=1 \
python3 \
"$DBG/uav2_demo5_waypoints.py" \
> >(tee "$DBG/demo5_waypoints.log") \
2>&1 &

PREVIEW_PID=$!

sleep 1

if ! kill -0 "$PREVIEW_PID" 2>/dev/null; then
    echo
    echo "[ABORT] tangent mission exited immediately:"
    cat "$DBG/demo5_waypoints.log"
    exit 1
fi

MISSION_RET=0

wait "$PREVIEW_PID" || MISSION_RET=$?

if [ "$MISSION_RET" -ne 0 ]; then
    echo
    echo "[FAIL] Demo9 waypoint mission returned $MISSION_RET"
    tail -n 100 "$DBG/demo5_waypoints.log"
    wait_safe_pilot_takeover
    exit 1
fi

echo
echo "========================================"
echo " DEMO9 MULTI-WAYPOINT MISSION COMPLETE"
echo "========================================"
echo
