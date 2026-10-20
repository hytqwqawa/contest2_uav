# Demo7 Development v1 — Demo5 mixed POINT/EGO mission + continuous QR

**Development only, not flight accepted.**

Source baseline: `demo/05_mixed_point_ego_v2_accepted` (unchanged).

- Original flight program unmodified: P1 (F1,L1,POINT yaw hold) → P2 (F3,L−1,EGO avoidance) → P3 (F5,L0,EGO) → POINT landing.
- Demo7 starts an independent subscriber at mission startup. It decodes D435i **RGB** at max 5 Hz, requires 3 successive frames of same unambiguous QR, and **prints the fresh QR payload in bold yellow at 1 Hz** while PX4 is ARMED + OFFBOARD. If no QR is visible, it prints nothing. It never sends setpoints or flight service commands.
- QR also publishes `/contest2/demo7/qr/result` and `/contest2/demo7/qr/status` (nonlatched); invalid/stale/occluded QR resets.
- Terminal 2 uses D435i RGB **640×480 30 Hz** and keeps depth **640×480 30 Hz**, plus original 0.20 m/s EGO configuration.
- QR observer errors do **not** trigger waypoint changes, aborts or landing; pilot safety and flight logic remain with the existing proven Demo5 source. If the observer stops, its own QR output stops; do not assume results keep working.
- ANSI yellow requires a terminal displaying ANSI colors. Scanned control characters are JSON-escaped before display.

## Install

On UAV, after uploading `demo7_qr_overlay_patch_v1.tar.gz`:

```bash
cd ~
tar -xzf demo7_qr_overlay_patch_v1.tar.gz
bash demo7_qr_overlay_patch_v1/install_demo7.sh
cd ~/contest2_uav/dev/projects/07_demo5_live_qr_v1
./check_demo7_static.sh
```

## Ground first (all DISARM/POSCTL)

Terminal 1: `./start_core.sh`

Terminal 2: `./start_backend.sh`

Terminal 3: `./test_demo7_qr_ground.sh` — hold QR in view; expect bold yellow output once each second while visible. Press Ctrl+C when ground test finished.

Then stop the QR ground test, hold QR steadily visible, and run `./test_demo7_camera_load.sh`: it requires actual QR detection while Depth/Color/ODOM all stay >=25 Hz. Do NOT run `run_demo7.sh` until initial ground inspection passes.

## Three-terminal flight entrypoints (later, after preflight inspection)

Terminal 1: `./start_core.sh`

Terminal 2: `./start_backend.sh`

Terminal 3: `./check_demo7_static.sh && ./run_demo7.sh`

`run_demo7.sh` starts a lightweight flight rosbag; `start_demo7.sh` runs the QR observer and calls the original `start_demo5.sh` unmodified. Original RViz, Demo Frame, CH5/6/7/8 authorization, waypoint and landing behavior is preserved.

Preflight state CH5 DISARM, CH6 POSCTL, CH7 LOW, CH8 SAFE. No automatic ARM; never Ctrl+C or DISARM while airborne. For abnormal flight revoke CH7 and manually take over with CH6 POSCTL as trained. Do not test QR in place of the pilot safety workflow. POINT P1 flight is straight-line, no obstacle avoidance.

## After flight

Review the original `demo5_waypoints.log` and rosbag at `~/contest2_uav/data/bags/demo7_qr_overlay_v1/`. Freeze into `demo/` only after complete real-flight success and evidence review.
