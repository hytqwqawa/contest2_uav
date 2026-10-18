# contest2 Demo05 — mixed POINT/EGO + D435i QR overlay

This is an **adaptation** of contest1's accepted Demo7 snapshot, not yet accepted or flight-tested on contest2.
The original `07_demo5_live_qr_v1_accepted.zip` is left untouched.

## Installed location
`~/contest2_uav/demo/05_mixed_point_ego_qr/`

## Original functional behavior retained
* P1: forward 1 m, left 1 m, straight POINT (fixed yaw, **NO obstacle avoidance**)
* P2: forward 3 m, left -1 m, EGO obstacle-aware trajectory/tangent yaw
* P3: forward 5 m, left 0 m, EGO then original POINT landing
* Flight height 0.8 m; EGO & POINT speed 0.20 m/s
* QR observer reads **D435i RGB** `/contest2/camera/color/image_raw`, 640x480@30fps
* pyzbar QR-only decode <=5 Hz; 3 consistent frames; yellow output 1 Hz only with ARMED+OFFBOARD, independent of flight commands
* QR topics: `/contest2/demo5/qr/result`, `/contest2/demo5/qr/status` (std_msgs/String)
* CH5 physical ARM/DISARM; CH6 POSCTL; CH7 autonomy authorization; CH8 kill SAFE; no software ARM/force-disarm
* Demo5 first gap **<0.10m** at both gates; touchdown 0.08m unchanged.

## Required independent hardware validation
* `ROS_IP=192.168.8.102`, MAVROS `uav_id=2`, D435i serial `939722070209` based on previously supplied contest2 snapshot. Confirm actual current devices locally.
* Demo frame is captured from the **nose direction at mission start**. P1 is a non-avoiding diagonal POINT segment. Each leg and fence must be checked physically for contest2's real flight field.
* The accepted contest1 runtime snapshot differs from contest2's runtime OFFBOARD manager (post-arm wait and AUTO.LAND added on contest2). A private copy of the **accepted logic** is included as `offboard_manager_demo05.py`, launched only by this Demo5; current contest2 runtime remains untouched.
* Other runtime nodes are reused from contest2; they matched the accepted snapshot after contest1/contest2 namespace adaptation in the archived engineering files, but current host installation must be verified.
* `README_DEMO7.md` or README in original source described earlier development. Current accepted logic is verified in the source flight evidence, not inherited as a contest2 flight acceptance.

## Preliminary steps (DISARM, props off, no flight)
1. Run `./check_demo7_static.sh` after installing package; requires Python3 yaml (and on runtime rospy, cv_bridge, pyzbar/zbar).
2. Check directory and ROS dependencies. Core T1, backend T2, QR ground test `./test_demo7_qr_ground.sh` T3; check RGB+Depth+ODOM rates with `./test_demo7_camera_load.sh`. Stop ground observer before mission.
3. Inspect RViz waypoint markers, actual field clearance, local pose consistency and pilot controls; flight requires on-site preflight/physical safety validation.

## Three terminals (only after ground checks and physical safety verification)
T1 `~/contest2_uav/demo/05_mixed_point_ego_qr/start_core.sh`
T2 `~/contest2_uav/demo/05_mixed_point_ego_qr/start_backend.sh`
T3 `~/contest2_uav/demo/05_mixed_point_ego_qr/run_mission.sh`

To reproduce the **original script naming**, T3 also accepts `./check_demo7_static.sh && ./run_demo7.sh`.
`run_mission.sh` is a wrapper to run the SAME QR overlay (not the bare original Demo5 mission).

## Scope
The archive contains **only Demo05 files**, never copied build/devel runtime, prior logs/bags or the frozen upstream accepted hashes. Do not overwrite `~/contest2_uav/runtime` or contest2 Demo1–4.

This package passed offline source syntax and matching checks on the supplied snapshot only. It does NOT prove contest2 can fly it safely.
