# contest2 Demo04 — 5 m pole (contest1 proven logic migrated)

**Status: source-migration static review only. NOT flight-validated on contest2.**
The source is derived from user-provided contest1 Demo04, reported flight-proven
by the user. The contest2-specific build has not been run against live PX4/ROS
or flown. Do not assume safety or successful operation until checked on contest2.

## Three formal entrypoints (contest2 computer only)

Terminal 1: `~/contest2_uav/demo/04_5m_pole/start_core.sh`
Terminal 2: `~/contest2_uav/demo/04_5m_pole/start_backend.sh`
Terminal 3: `~/contest2_uav/demo/04_5m_pole/run_mission.sh`

## Architecture

- T265 raw → nan guard → filtered → **Demo04 local landing sanitizer**
  → vision_pose_node (uav_id=2) → PX4 → `/contest2/mavros/local_position/odom`.
- D435i (contest2 serial `939722070209`) → EGO planner → traj_server.
- POINT/EGO → trajectory_mux → trajectory_interface → MAVROS → PX4.
- Demo04 local mode manager carries over the contest1-proven behavior, *not*
  contest2 existing post-arm delay or AUTO.LAND manager. This is a separate
  Python process started/stopped by Demo04 `start_core.sh`. It does not replace
  any installed contest2 files.

## Preserved parameters

- Target: demo-frame forward=0, left=+5.00 m, height=0.80 m.
- `demo_frame.yaml` is a field-orientation calibration and must be checked
  against contest2's actual obstacle course while DISARMED. The provided
  file matches the existing contest2 Demo1 snapshot. Current origin is
  captured afresh at startup; basis direction is loaded from the YAML.
- Fence: forward [-1.50,+1.50], left [-0.50,+5.50], height [-0.25,+1.30].
- EGO max_vel=0.20 m/s.
- **First gap strictly <0.08 m.** Direct execution deadline=60 s.
- Arrival: dXY <0.15 m, dZ <0.10 m, speed <0.20 m/s, stable ≥0.80 s.
- Demo3 POINT takeoff: height=0.80 m, speed=0.40 m/s, acceleration=0.80 m/s².
- Demo3 POINT/OFFBOARD landing till touchdown: touchdown range=0.08 m,
  timeout=0.30 s, stable=0.20 s, min_drop=0.30 m. Then POSCTL and PX4
  land detector; no AUTO.LAND handoff, no software auto-ARM, no force-disarm.

## Namespace and hardware

- All project business topics under `/contest2`.
- ROS master `192.168.8.102:11311`, contest2 ROS_IP `192.168.8.102`.
- MAVROS uav_name=contest and uav_id=2.
- Uses contest2's existing `runtime/env.sh`, `sunray_ws`, `Fast-Drone-250`,
  `ego_uav_ws` and their existing camera extrinsic/EGO launch configuration.
- Does not copy or replace contest2 Demo1, Demo2, Demo3 or runtime contents.

## Deployment and commissioning

Deploy only on contest2 PC, after confirming `~/contest2_uav/demo/04_5m_pole`
does not already exist. Archive path is `contest2_uav/demo/04_5m_pole/...`.
Deployment example, after copying the tarball into `$HOME/Downloads`:

```bash
if [ -e "$HOME/contest2_uav/demo/04_5m_pole" ]; then
  echo "STOP: Demo04 already exists; do not overwrite it."
else
  tar -xzf "$HOME/Downloads/contest2_demo04_migration_20261008.tar.gz" -C "$HOME"
fi
```

Before any ARM/flight, establish contest2 DISARM ground-pass evidence for T1,
T2, T3, all sensor/FCU/RC diagnostics and physical field safety, following the
one-step-at-a-time procedure. No automatic ARM/force-disarm. CH5 physical ARM,
CH6 POSCTL takeover, CH7 mission authorization, CH8 kill only for emergencies.
Never stop operational terminals while airborne.

**Important:** Hand-off tarballs omit most actual ROS workspace contents and
scripts (including `runtime/common/scripts/uav2_system_ready_node.py`). They
were not available for offline runtime validation. The exact hardware wiring,
external PX4 estimator setup, ROS packages and flight behavior must be verified
on contest2; a static source audit alone is insufficient.
