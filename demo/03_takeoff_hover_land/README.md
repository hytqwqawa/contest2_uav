# 03_takeoff_hover_land

Minimal real-flight demo:

    TAKEOFF -> HOVER -> LAND

No RViz.
No D435.
No EGO planner.

## Formal launch

Terminal 1:

    ~/contest2_uav/runtime/start_core.sh

Terminal 2:

    ~/contest2_uav/demo/03_takeoff_hover_land/run_demo.sh

## Pilot controls

Initial:

- CH5 = DISARM
- CH6 = POSCTL
- CH7 = LOW

When the mission prints WAITING FOR PILOT:

1. CH7 LOW -> MID -> HIGH
2. Keep CH6 in POSCTL
3. CH5 -> ARM

Do not move CH6/CH7 during autonomous flight.

No software ARM or forced software DISARM is used.

## Flight

- Takeoff height: 0.80 m
- Hover time: 5.0 s
- Landing uses the accepted POINT terminal descent.
- Touchdown threshold remains 0.08 m.

## Demo3 touchdown experiment

Demo1/Demo2 currently transition from OFFBOARD to POSCTL
after touchdown while the vehicle may still be ARMED.

Demo3 intentionally does NOT make that ARMED mode transition.

During normal landing, POINT controls the descent only until
the fresh downward range reaches 0.10 m.

At that point Demo3 requests PX4 native AUTO.LAND through
offboard_manager and stops its own trajectory control after
PX4 confirms AUTO.LAND.

PX4 then owns the final few centimeters, land detection and
COM_DISARM_LAND automatic motor disarm.

The strict 0.08 m touchdown threshold is unchanged and is
retained as a fallback if AUTO.LAND handoff fails.

If AUTO.LAND or PX4 auto-disarm does not complete, the
script asks the pilot to use physical CH5 DISARM.

No software force-disarm is performed.
