#!/usr/bin/env bash
# Offline inspection ONLY. Never starts ROS / requests OFFBOARD / arms.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONDONTWRITEBYTECODE=1
python3 -B "$D/demo5_config.py" --check "$D/demo5_waypoints.yaml"
python3 -B "$D/test_demo5_offline.py"
python3 -B "$D/test_demo7_offline.py"
python3 -B "$D/test_demo8_offline.py"
python3 -B "$D/test_demo8_controller_offline.py"
python3 -B - "$D" <<'PYAST'
import ast,sys,py_compile,tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
p=Path(sys.argv[1])
with tempfile.TemporaryDirectory() as td:
    for i,f in enumerate(sorted(p.glob('*.py'))):
        ast.parse(f.read_text(),filename=str(f))
        py_compile.compile(str(f),cfile=str(Path(td) / ('src_%d.pyc' % i)),doraise=True)
for f in sorted(p.glob('*.launch')):
    ET.parse(f)
print('[ OK ] Python py_compile / AST and launch XML parsed')
PYAST
for f in "$D"/*.sh; do bash -n "$f"; done
printf '%s\n' '[ OK ] Demo8 STATIC/OFFLINE PASS. Does NOT authorize flight.'
