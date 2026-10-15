#!/usr/bin/env python3
"""Offline acceptance of non-controlling flight-wide QR overlay."""
import ast
import sys
from pathlib import Path

R = Path(__file__).resolve().parent
monitor = (R / 'demo7_qr_monitor.py').read_text()
tree = ast.parse(monitor)
for dangerous in ('ServiceProxy', 'Publisher("/contest2/mavros/setpoint_raw/local',
                  'Publisher("/contest2/ego_uav_interface/point_goal'):
    if dangerous in monitor:
        raise SystemExit('[FAIL] QR monitor must never command flight: ' + dangerous)
assert '_ground_test:=false' in (R / 'start_demo7.sh').read_text()
assert '"$D/start_demo5.sh"' in (R / 'start_demo7.sh').read_text()
assert '\\033[1;33m' in monitor and 'rospy.Duration(1.0)' in monitor
assert 'max_vel:=0.20' in (R / 'start_backend.sh').read_text()
assert 'enable_color:=true' in (R / 'start_backend.sh').read_text()
assert 'enable_depth:=true' in (R / 'start_backend.sh').read_text()
assert 'color_fps:=30' in (R / 'start_backend.sh').read_text()
assert 'point_setpoint' not in monitor and 'use_ego' not in monitor
sys.path.insert(0, str(R))
from qr_stability import QrStability
qr = QrStability(min_frames=3, freshness_s=1.5)
qr.process(['HIT'], 0.0)
qr.process(['HIT'], 0.2)
assert qr.current(0.2) == ''
qr.process(['HIT'], 0.4)
assert qr.current(0.4) == 'HIT'
assert qr.current(2.1) == ''
qr.process([], 2.2)
assert qr.current(2.2) == ''
qr.process(['A', 'B'], 2.4)
assert qr.current(2.4) == '' and qr.status(2.4) == 'AMBIGUOUS'
assert 'ZBarSymbol.QRCODE' in monitor
print('[ OK ] Demo7 QR 5Hz/3-frame/1Hz yellow, freshness, ambiguity checks')
print('[ OK ] QR monitor is read-only; no flight source fingerprint gate')
print('[ OK ] Demo7 observer contains no flight services or setpoint publishers')
