# Demo 5 - ordered EGO multi-waypoint mission

**Status: DEVELOPMENT CANDIDATE, NOT FLIGHT ACCEPTED.**

Copied from the exact uploaded Demo4 snapshot, preserving the Demo3 POINT takeoff / V4 landing implementation, T265 NaN guard and landing sanitizer, RC CH5/6/7/8 safety and EGO max velocity 0.20 m/s.

## Frame and initial aircraft heading

At DISARMED mission start, **physically orient the NOSE toward the NEW field FORWARD**, which is the OLD Demo4 LEFT. The NEW LEFT is the opposite direction from the OLD Demo4 FORWARD. Never rotate the basis a second time in software: the ground Demo Frame Guard already defines +F = captured nose yaw and +L = 90 deg left of it. The snapshot `demo_frame.yaml` is only an initial format/basis check; the actual world basis is captured anew into `session_frame.yaml` every flight.

Authoritative Demo fence in new frame: forward [-1.00,+6.00] m, left [-2.00,+2.00] m, height [-0.25,+1.30] m. It is enforced by the existing Demo-frame aircraft-position guard and by new **sampled-path checks**, including a new B-spline before every leg and in-flight EGO replans. It is NOT a physical hard geofence; the MAVROS trajectory interface has its separate broad world AABB. Pilot must retain manual takeover. The camera/obstacle pipeline must be verified in the real environment.

## Edit the ordered goals

Change only `demo5_waypoints.yaml` when DISARMED and the Demo5 Terminal 3 is stopped:

```yaml
waypoints:
  - {forward: 2.0, left:  1.0}  # P1
  - {forward: 4.0, left: -1.0}  # P2
  - {forward: 5.0, left:  0.0}  # P3 final -> land
```

Add/remove rows as needed; **the last one is always the only landing destination.** Goals are converted into WORLD using the *current* captured Demo session basis. EGO is used on every inter-waypoint leg, including in-flight replanning for obstacles. Between legs the proven controller enters a POINT HOLD before validating and releasing a fresh EGO trajectory. There are no POINT straight-line waypoint transfers. 

Geometry fence is fixed for this revision; goal margin must be >=0.10m. Reused POINT takeoff height is fixed at 0.80m, so configuration validator deliberately rejects attempts to change it without updating the upstream flight controller. Segments have a 90s default timeout and require Demo4's dXY/dZ first-gap <0.08m before and after trajectory trigger.

## Ground-only checks

```bash
cd ~/contest2_uav/demo/05_multi_waypoint_ego
python3 -B demo5_config.py --check demo5_waypoints.yaml
./check_demo5_static.sh
```

## Three original terminals - only AFTER ground checks pass

Terminal 1 `./start_core.sh`; Terminal 2 `./start_backend.sh`; Terminal 3 `./run_mission.sh`. Terminal 3 shows all waypoint markers and requires visual inspection before physical pilot authorization (no text confirmation). No software ARM / DISARM. Manual CH5 ARM/DISARM; CH6 manual POSCTL; CH7 autonomous authorization; CH8 emergency.

After stable takeoff: P1 via EGO -> POINT HOLD -> P2 via new EGO trajectory -> POINT HOLD -> P3 via new EGO trajectory -> proven POINT landing. Unsafe path, stale setpoint, RC/health/guard violation, or first-gap >=0.08m is fail-closed and requires safe operator handling; no automatic skipping to the next waypoint.

**Flight-validation status:** No PX4 / ROS live integration or actual flight verification has been performed on this Demo5 candidate. First test with DISARMED RViz/session checks and ROS topic checks, then controlled trials only after the previous step is passed. Avoid any unattended autonomous flight.
