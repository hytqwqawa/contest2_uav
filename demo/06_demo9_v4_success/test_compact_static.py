#!/usr/bin/env python3
"""No ROS, no ARM, no motors. Static integrity checks for compact bundle."""
import ast
import subprocess
from pathlib import Path
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parent


def check():
    for name in ('start_core.sh', 'start_backend.sh', 'run_demo10.sh'):
        p = ROOT/name
        assert p.is_file(), name
        subprocess.run(['bash','-n',str(p)], check=True)
    for src in ROOT.rglob('*.py'):
        ast.parse(src.read_text(), filename=str(src))
    for xml in ROOT.rglob('*.launch'):
        ElementTree.parse(xml)
    for xml in ROOT.rglob('package.xml'):
        ElementTree.parse(xml)
    assert (ROOT/'ros_src/ego_uav_interface/scripts/yaw_guard_core.py').exists()
    s = (ROOT/'run_demo10.sh').read_text()
    assert 'pilot_takeoff.py' in s and 'uav2_demo5_waypoints.py' in s
    assert 'start_demo5.sh' not in s and 'run_demo9.sh' not in s
    assert 'rostopic echo' in s and 'session_frame.yaml' in s
    assert 'demo7_qr_monitor' not in s
    src = (ROOT/'uav2_demo5_waypoints.py').read_text()
    assert "self.root / 'demo9_posctl_handoff.py'" in src
    assert 'DEMO10_LANDING_MODE' not in src
    assert 'point_status == "AIRBORNE_HOLD"' in src
    assert 'stable >= 0.30' in src
    assert 'self.demo9_verified_hold(' not in src
    land = (ROOT/'demo9_posctl_handoff.py').read_text()
    assert "custom_mode='POSCTL'" in land
    assert 'armed=False' not in land and 'arming' not in land
    for source in (ROOT/'demo8_interface_overlay/scripts/trajectory_interface_demo8.py',
                   ROOT/'ros_src/ego_uav_interface/scripts/trajectory_mux_v4_only.py'):
        assert source.exists()
    print('[PASS] shell, Python AST, ROS XML, endpoint chain and landing wiring')


if __name__ == '__main__':
    check()
