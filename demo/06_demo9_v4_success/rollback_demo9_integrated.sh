#!/usr/bin/env bash
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT="$HOME/contest2_uav"
[[ "$D" == "$ROOT/demo/09_demo8_posctl_integrated_v1" ]] || { echo '[STOP] Wrong install directory'; exit 2; }
source "$ROOT/runtime/env.sh"
RAW="$(timeout 4s rostopic echo -n 1 /contest2/mavros/state 2>/dev/null || true)"
if grep -Eq '^armed:[[:space:]]*True' <<<"$RAW";then
  echo '[STOP] Aircraft armed. Land/DISARM before rollback.';exit 2
fi
NODES="$(timeout 5s rosnode list 2>/dev/null || true)"
for NODE in /contest2/uav2_demo5_waypoints /contest2/demo04_5m_takeoff_gate \
            /contest2/demo9_integrated_posctl_handoff /contest2/point_traj_node;do
  if grep -Fxq "$NODE" <<<"$NODES";then
    echo "[STOP] ROS node still active: $NODE. Stop Core/Backend after DISARM."; exit 2
  fi
done
OUT="$HOME/uav_handoff_exports/demo9_integrated_removed_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$(dirname "$OUT")"
mv -- "$D" "$OUT"
echo "[OK] Moved integrated Demo9 to $OUT; Demo8/V3/V13/runtime untouched"
