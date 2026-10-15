# 02_waypoint_pole

Status: REAL-FLIGHT ACCEPTED
Date: 2026-10-03

## Purpose

Waypoint obstacle-avoidance demonstration:

P0 -> P1 -> P2 -> P3 -> LAND

Demo-frame coordinates:

- P1: F=+1.00, L=+1.00
- P2: F=+0.50, L=+3.00
- P3: F=-0.50, L=+4.00
- Pole: F=+1.00, L=+2.00
- Flight height: 0.80 m

## Safety

- NO SOFTWARE ARM
- CH5: manual ARM / DISARM
- CH6: manual POSCTL takeover
- CH7: autonomy permission
- CH8: emergency only
- First-gap limit remains 0.08 m
- Do not loosen first-gap protection

## Accepted launch procedure

Terminal 1:

    ~/contest2_uav/runtime/start_core.sh

Terminal 2:

    ~/contest2_uav/demo/02_waypoint_pole/start_backend.sh

Confirm RViz and backend READY.

Terminal 3:

    ~/contest2_uav/demo/02_waypoint_pole/run_mission.sh

## Runtime-generated file

session_frame.yaml is generated fresh by start_backend.sh and is not part
of the frozen source configuration.

## Development rule

Do not experimentally modify this validated demo in place.

Keep experimental variants outside this formal demo directory.

After successful ground and real-flight validation, changes may be promoted
back into demo/.
