# 01_4m_pole

Status: REAL-FLIGHT ACCEPTED
Date: 2026-10-03

## Mission

Fixed takeoff point -> POINT takeoff -> EGO obstacle avoidance ->
tangent-yaw execution -> 4 m target -> proven V4 landing.

Nominal Demo-frame geometry:

- Target: F=0.0 m, L=+4.0 m
- Flight height: about 0.8 m
- Pole: F≈0.0 m, L≈+2.4 m

## Accepted launch procedure

Terminal 1:

    ~/contest2_uav/runtime/start_core.sh

Terminal 2:

    ~/contest2_uav/demo/01_4m_pole/start_backend.sh

Terminal 3:

    ~/contest2_uav/demo/01_4m_pole/run_mission.sh

## Safety

- NO SOFTWARE ARM
- CH5: manual ARM / DISARM
- CH6: manual POSCTL takeover
- CH7: autonomy permission
- CH8: emergency only
- First-gap threshold remains 0.08 m
- Proven V4 landing retained
- V5/V6 soft-Z experiments are NOT part of this accepted demo

## Development rule

Do not experimentally modify this directory in place.

Keep experimental variants outside this formal demo directory.
