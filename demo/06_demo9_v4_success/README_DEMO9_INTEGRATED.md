# Demo9 Integrated: Demo8 mixed EGO/POINT + V3 takeoff + V13 POSCTL landing

Directory: `~/contest2_uav/demo/09_demo8_posctl_integrated_v1/`

This is a NEW independent copy of the installed Demo8. The accepted Demo8,
Demo9 V3/V13 and runtime source tree are never changed.

## Exact mission

1. Original Demo8 Demo-frame capture, RViz waypoint preview, EGO yaw/QR observer.
2. Exact Demo9 V3 takeoff gate and TAKEOFF-only flight operations copied at install.
3. Independent altitude verification from Demo8, plus 3 seconds of POINT HOLD.
4. All Demo8 YAML POINT/EGO legs, EGO replanning and terminal yaw preserved.
5. At the FINAL waypoint only, restore POINT HOLD and hold 3 seconds.
6. Request POSCTL once using the same V13 fresh/valid real RC calibration rules,
   but no strict low-throttle/center-stick condition.
7. The human pilot uses the REAL throttle to descend; PX4 stays responsible for
   native landing detection, and physical CH5 is available for DISARM on ground.

THIS IS NOT autonomous touchdown. No virtual RC, AUTO.LAND, software ARM/DISARM,
or range-based terminal motor cut is included. The rangefinder <0.40m data
must never be used for motor-stop detection.

## Three terminals, like Demo8

Terminal 1:
```
bash "$HOME/contest2_uav/demo/09_demo8_posctl_integrated_v1/start_core.sh"
```
Terminal 2:
```
bash "$HOME/contest2_uav/demo/09_demo8_posctl_integrated_v1/start_backend.sh"
```
Terminal 3 (starts bag recording, QR and Demo8 mission):
```
bash "$HOME/contest2_uav/demo/09_demo8_posctl_integrated_v1/run_demo9.sh"
```

No typed READY phrase and no separate `check` command before every start.
The preserved Demo8 physical gate still REQUIRES CH5/6/7/8, PX4 health,
valid odometry and session fence. Initial state: DISARM, POSCTL, CH7 LOW,
CH8 SAFE. Check the RViz points and obstacles BEFORE physical arming.
Only the pilot can choose to authorize flight on the real aircraft.

When mode changes to POSCTL, the real RC throttle controls vertical motion.
A middle/high throttle may cause hover/ascent instead of descent. Keep pilot
in control until contact and physically DISARM using the approved procedure.

## Notes

- `demo5_waypoints.yaml` is copied unchanged from the installed Demo8.
  Review its actual POINT/EGO destinations and final landing area for THIS
  flight. POINT straight paths do NOT have obstacle avoidance.
- For failures, CH7 LOW and CH6 POSCTL remain the physical pilot override;
  never use Ctrl+C as an in-flight emergency stop.
- This integrated combination is not yet flown. Offline tests are NOT an
  authorization for flight; first do a powered ground check and controlled
  low-risk validation with a pilot/observer.
- Flight ROS bags: `~/contest2_uav/data/bags/demo9_demo8_posctl_integrated_v1/`.
- No PX4 settings changed and no artificial battery bypass.

## Offline regression (run once after install or modifications)
```
bash "$HOME/contest2_uav/demo/09_demo8_posctl_integrated_v1/check_demo8_static.sh"
python3 -B "$HOME/contest2_uav/demo/09_demo8_posctl_integrated_v1/test_demo9_integrated.py"
```

## Rollback (DISARM and stop three terminals first)
```
bash "$HOME/contest2_uav/demo/09_demo8_posctl_integrated_v1/rollback_demo9_integrated.sh"
```
