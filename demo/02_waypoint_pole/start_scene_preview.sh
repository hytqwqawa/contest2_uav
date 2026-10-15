#!/usr/bin/env bash

set -e

ROOT="$HOME/contest2_uav"
DEMO="$ROOT/demo/02_waypoint_pole"

source "$ROOT/runtime/env.sh"

export ROS_NAMESPACE=/contest2

echo
echo "========================================"
echo " UAV2 WAYPOINT-POLE SCENE PREVIEW"
echo "========================================"
echo
echo "Required:"
echo "  aircraft at fixed takeoff point"
echo "  DISARMED"
echo "  CH7 LOW"
echo
echo "NO FLIGHT COMMANDS WILL BE SENT."
echo

python3 \
"$DEMO/uav2_waypoint_scene_preview.py" &

SCENE_PID=$!

cleanup()
{
    kill -INT "$SCENE_PID" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

sleep 2

if ! kill -0 "$SCENE_PID" 2>/dev/null; then
    echo "[ABORT] scene preview exited"
    exit 1
fi

rviz \
-d "$DEMO/waypoint_pole_demo.rviz" \
/tf:=/contest2/tf \
/tf_static:=/contest2/tf_static

wait
