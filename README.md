# contest2_uav

Unified UAV workspace for contest2.

## Directory layout

runtime/
    Stable common ROS/PX4 runtime.
    Do not use for experimental modifications.

demo/
    Real-flight accepted demonstrations.

    02_waypoint_pole/
        REAL-FLIGHT ACCEPTED 2026-10-03

dev/
    All active development and experimental variants.

data/
    bags/
    logs/
    calibration/

## Stable runtime

Terminal 1:

    ~/contest2_uav/runtime/start_core.sh

Runtime stack:

    runtime/sunray_ws
    runtime/Fast-Drone-250
    runtime/ego_uav_ws

The runtime has passed ground integration validation with:

- T265 raw ~200 Hz
- T265 filtered ~200 Hz
- PX4 local odometry ~30 Hz
- MAVROS
- vision pose
- NaN guard
- state monitor
- preflight monitor
- RC supervisor
- OFFBOARD manager

## Development policy

TEST -> REAL-FLIGHT ACCEPTED -> DEMO

Do not experimentally modify a validated demo in place.
Keep experimental work outside the formal demo directory.


Do not delete legacy directories yet.
