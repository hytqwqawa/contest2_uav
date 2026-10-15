#!/usr/bin/env bash

set -u

ROOT="$HOME/contest2_uav"
DEMO="$ROOT/demo/03_takeoff_hover_land"

source "$ROOT/runtime/env.sh"

export ROS_NAMESPACE=/contest2

LAUNCH_PID=""

vehicle_arm_state()
{
    local out

    out="$(
        timeout 2 \
        rostopic echo -n 1 \
        /contest2/mavros/state \
        2>/dev/null || true
    )"

    if printf '%s\n' "$out" | \
       grep -Eq '^[[:space:]]*armed:[[:space:]]*True'; then
        echo "ARMED"
        return
    fi

    if printf '%s\n' "$out" | \
       grep -Eq '^[[:space:]]*armed:[[:space:]]*False'; then
        echo "DISARMED"
        return
    fi

    echo "UNKNOWN"
}


cleanup()
{
    trap - EXIT INT TERM

    if [ -z "$LAUNCH_PID" ] || \
       ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        return
    fi

    local state
    state="$(vehicle_arm_state)"

    if [ "$state" = "ARMED" ]; then

        echo
        echo "========================================"
        echo " SAFETY: VEHICLE STILL ARMED"
        echo " POINT backend will NOT be stopped."
        echo
        echo " If airborne:"
        echo "   CH7 -> LOW"
        echo "   CH6 -> POSCTL"
        echo "   keep CH5 ARM until safe."
        echo
        echo " After safe landing:"
        echo "   CH5 -> DISARM"
        echo "========================================"

        while true; do

            state="$(vehicle_arm_state)"

            if [ "$state" = "DISARMED" ]; then
                echo "[ OK ] Vehicle DISARMED; backend may stop."
                break
            fi

            sleep 0.25
        done

    elif [ "$state" = "UNKNOWN" ]; then

        echo
        echo "[SAFETY] MAVROS armed state is UNKNOWN."
        echo "         Leaving POINT backend running."
        echo "         Do not kill it until vehicle state is known safe."
        return
    fi

    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
}

trap cleanup EXIT INT TERM


echo
echo "========================================"
echo " DEMO3 - TAKEOFF / HOVER / LAND"
echo "========================================"
echo
echo "Required:"
echo "  Terminal 1:"
echo "    ~/contest2_uav/runtime/start_core.sh"
echo
echo "Initial RC:"
echo "  CH5 = DISARM"
echo "  CH6 = POSCTL"
echo "  CH7 = LOW"
echo
echo "NO RViz"
echo "NO D435"
echo "NO EGO planner"
echo "NO SOFTWARE ARM / DISARM"
echo


if ! rosnode list >/dev/null 2>&1; then
    echo "[ABORT] ROS master/core is not available."
    exit 1
fi


if ! rosnode list 2>/dev/null | \
     grep -Fxq /contest2/trajectory_interface; then

    echo "[ABORT] /contest2/trajectory_interface missing."
    echo "        Start runtime/start_core.sh first."
    exit 1
fi


for N in \
    /contest2/point_traj_node \
    /contest2/trajectory_mux \
    /contest2/takeoff_hover_land_demo
do
    if rosnode list 2>/dev/null | grep -Fxq "$N"; then
        echo "[ABORT] Existing demo node: $N"
        echo "        Stop Demo1/Demo2/Demo3 backend first."
        exit 1
    fi
done


echo "[1/2] Starting minimal POINT backend..."

setsid roslaunch \
    "$DEMO/minimal_point_interface.launch" &

LAUNCH_PID=$!


deadline=$((SECONDS + 8))

while [ "$SECONDS" -lt "$deadline" ]; do

    if \
        rosnode list 2>/dev/null \
            | grep -Fxq /contest2/point_traj_node \
        && \
        rosnode list 2>/dev/null \
            | grep -Fxq /contest2/trajectory_mux
    then
        break
    fi

    sleep 0.2
done


if ! rosnode list 2>/dev/null \
    | grep -Fxq /contest2/point_traj_node; then

    echo "[ABORT] point_traj_node failed to start."
    exit 1
fi


if ! rosnode list 2>/dev/null \
    | grep -Fxq /contest2/trajectory_mux; then

    echo "[ABORT] trajectory_mux failed to start."
    exit 1
fi


echo "[ OK ] Minimal POINT backend ready"
echo
echo "[2/2] Starting Demo3 mission..."
echo

PYTHONUNBUFFERED=1 \
python3 "$DEMO/takeoff_hover_land.py"

RET=$?

if [ "$RET" -eq 0 ]; then
    echo
    echo "========================================"
    echo " DEMO3 FINISHED"
    echo "========================================"
else
    echo
    echo "[FAIL] Demo3 mission exited with code $RET"
fi

exit "$RET"
