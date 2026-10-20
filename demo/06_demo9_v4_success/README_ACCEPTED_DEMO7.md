# Demo7 — ACCEPTED: Mixed POINT/EGO + continuous yellow QR

Status: successful real flight reported by pilot, corroborated by the recorded multi-waypoint completion, PX4 disarm, and in-flight QR ROS topics.

- P1: F+1 L+1 POINT, holds yaw, NO obstacle avoidance
- P2: F+3 L-1 EGO with obstacle-aware trajectory and tangent yaw
- P3: F+5 L0 EGO, then original POINT landing
- D435i RGB 640x480@30Hz + depth 640x480@30Hz
- QR observer: pyzbar at <=5Hz, >=3 consistent frames, yellow print at 1Hz while PX4 ARMED/OFFBOARD; no flight commands from observer
- No software ARM; original CH5/CH6/CH7/CH8 pilot safety remains
- POINT P1 is straight-line with no obstacle avoidance

Terminal 1: `./start_core.sh`
Terminal 2: `./start_backend.sh`
Terminal 3: `./check_demo7_static.sh && ./run_demo7.sh`

Evidence: `flight_evidence/mission.log` and `flight_evidence/FLIGHT_BAG.json`.
The bag remains in `data/bags/demo7_qr_overlay_v1/` and is verified by SHA256.
Runtime source snapshot is retained; compiled runtime workspaces are external.
Do not modify accepted source. For new development fork under `dev/`.
This archive does not independently prove real obstacle detours occurred.
