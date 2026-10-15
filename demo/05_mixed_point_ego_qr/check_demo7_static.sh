#!/usr/bin/env bash
# Offline code/config validation only; no ROS nodes started.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -B "$D/demo5_config.py" --check "$D/demo5_waypoints.yaml"
python3 -B "$D/test_demo7_offline.py"
python3 -B - "$D" <<'PY'
import ast, sys
import xml.etree.ElementTree as ET
from pathlib import Path
p = Path(sys.argv[1])
for fn in ('demo7_qr_monitor.py','qr_stability.py','test_demo7_offline.py'):
    ast.parse((p/fn).read_text())
for f in p.glob('*.launch'):
    ET.parse(f)
print('[ OK ] Demo7 Python and launch XML static validation')
PY
for f in "$D"/*.sh; do bash -n "$f"; done
printf '%s\n' '[ OK ] Demo7 shell syntax valid (NOT flight acceptance)'
