# Demo5 V2 -- MIXED POINT/EGO development branch

**Status: DEVELOPMENT ONLY. Not approved as an accepted flight demo.**

Original accepted Demo5 V1: `~/contest2_uav/demo/05_multi_waypoint_ego_accepted`.
This fork: `~/contest2_uav/dev/projects/05_mixed_point_ego_v2`.
`runtime/`, original `demo/05_multi_waypoint_ego/`, and the accepted snapshot are never modified by this installer.

## Edit only `demo5_waypoints.yaml`

```yaml
waypoints:
  - {forward: 1.0, left:  1.0, mode: POINT}
  - {forward: 3.0, left: -1.0, mode: EGO}
  - {forward: 5.0, left:  0.0, mode: EGO}  # final LAND
```

`mode: POINT` is a straight, quintic POINT trajectory; yaw stays at the heading measured at the beginning of **that leg**. It does NOT use obstacle avoidance. ONLY use POINT for a physically unobstructed segment, and inspect each straight path, not just its endpoint. The current POINT node rejects horizontal goals beyond 3 m actual distance. This development branch statically rejects configured POINT legs over **2.70 m** (reserve 0.30m for drift/arrival error) and aborts at runtime if actual distance exceeds 2.95 m. Intermediate POINT legs complete in `AIRBORNE_HOLD` and **do not LAND**.

`mode: EGO` preserves the successfully flown EGO avoidance/replanning, B-spline checks, 0.08m first-gap, TANGENT yaw, POINT HOLD between legs, and original landing chain.

The **last waypoint**, regardless of mode, is the only landing destination. Modes must be uppercase and supplied on **every** waypoint. Initial config keeps all existing successful waypoints as `mode: EGO` so behavior is not silently changed. You can change one segment to `POINT` when its path is clear.

Coordinate origin, heading, fence, initial height, T265 sanitizer, safety CH5/6/7/8, no-software ARM, Dashboard, OFFBOARD manager, and proven POINT landing are preserved. Horizontal POINT max_speed is explicitly configured 0.20 m/s in this fork's launch; EGO remains 0.20 m/s.

## Validation

Run `./check_demo5_static.sh` with aircraft DISARMED; then run offline/ground integration to verify POINT acceptance/status, no yaw change, EGO re-planning and POINT/EGO switching. **Static validation alone does NOT authorize flight**. Do not test a POINT segment with an obstacle in its straight path.

Once properly validated, the three terminal entries from this DEV directory are `./start_core.sh`, `./start_backend.sh`, and `./run_mission.sh`. Preserve manual RC CH6 takeover, CH7 LOW abort, and physical CH5 ARM/DISARM. There is no typed `CHECKED` prompt, but the pilot must visually verify RViz before authorization.
