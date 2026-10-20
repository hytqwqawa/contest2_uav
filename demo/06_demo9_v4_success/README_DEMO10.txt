DEMO10 SUCCESSFUL DEMO9 V4 FOUR-LEG COPY (not a formal flight safety certificate)
This folder is independent of the DEV V4 stage and original Demo9 source.
Flight route: EGO F3 L0 yaw0; EGO F3 L-1 yaw-90;
              EGO F2 L-1.5 yaw-180; POINT F1 L-1.5. Height 0.8m.
Terminal 1: bash ./start_core.sh
Terminal 2: bash ./start_backend.sh
Terminal 3: bash ./run_demo10.sh  (ACTUAL FLIGHT MISSION)
Before flight: DISARM/POSCTL/CH7 LOW, verify ground chain and RViz, pilot approval.
Pilot authorization, frame capture, fence, yaw/POINT hold and recovery
guards are unchanged. No automatic ARM is added.
OFFBOARD/position link interruption root cause remains unproven.
Never run V4 and Demo10 backend/mission simultaneously.
Temporary session_frame.yaml must regenerate at each new mission.
