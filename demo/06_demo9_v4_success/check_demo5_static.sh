#!/usr/bin/env bash
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$D"

if [ ! -f "$D/uav2_demo5_multi_waypoint_interface.launch" ]; then
    echo '[FAIL] Mixed-mode POINT/EGO interface launch missing'
    exit 1
fi

python3 -B "$D/demo5_config.py" --check "$D/demo5_waypoints.yaml"
python3 -B "$D/test_demo5_offline.py"

D4="$D" python3 -B - <<'PY'
import ast, os, xml.etree.ElementTree as ET
from pathlib import Path
p = Path(os.environ['D4'])
for f in p.glob('*.py'):
    ast.parse(f.read_text(),filename=str(f))
for f in p.glob('*.launch'):
    ET.parse(str(f))
mission = (p/'uav2_demo5_waypoints.py').read_text()
assert 'land_after=(index == total)' in mission
assert 'B-spline route violates authoritative Demo5 fence' in mission
assert 'post-trigger first gap >= 0.08 m' in mission
assert 'In-flight EGO replan' in mission
assert 'Demo5 YAML changed since DISARMED frame capture' in mission
assert 'self.execute_point_waypoint(i, total, wp)' in mission
assert 'self.plan_execute_waypoint(i, total, wp)' in mission
assert 'POINT goal not acknowledged as POINT_TRAJECTORY' in mission
assert 'POINT commanded yaw changed' in mission
assert 'self.start_land()' in mission
assert 'self.point_goal_pub.publish(msg)' in mission
assert 'NO software ARM' in mission
interface = (p/'uav2_demo5_multi_waypoint_interface.launch').read_text()
assert 'name="max_speed" value="0.20"' in interface
print('[ OK ] AST/XML and mixed-mode safety static assertions')
PY
for f in "$D"/*.sh; do
    bash -n "$f"
done
if grep -q 'type CHECKED' "$D/start_demo5.sh"; then
    echo '[FAIL] Obsolete typed CHECKED interaction found'
    exit 1
fi
# No old hard-coded active mission path.
if grep -nE '/demo/04_5m_pole|/demo/05_multi_waypoint_ego([/" ]|$)' \
    "$D/start_core.sh" "$D/start_backend.sh" "$D/run_mission.sh" \
    "$D/start_demo5.sh" "$D/uav2_demo5_waypoints.py"; then
    echo '[FAIL] Active source still references an old demo directory'
    exit 1
fi
echo '[ OK ] Demo5 V2 static checks finished (NOT a flight acceptance)'
