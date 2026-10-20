#!/usr/bin/env python3
"""Offline regression: preserve mixed Demo8 waypoint code, replace terminal landing."""
import ast
import hashlib
import os
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from rc_gate_v13 import is_handoff_ready


def node_method(name):
    root = ast.parse((ROOT / 'uav2_demo5_waypoints.py').read_text())
    cls = next(c for c in root.body if isinstance(c, ast.ClassDef) and c.name == 'TangentPreview')
    return next(f for f in cls.body if isinstance(f, ast.FunctionDef) and f.name == name)


class FakeRC:
    def __init__(self, throttle):
        self.channels=[1500,1500,1500,throttle]+[1000]*14


CAL={
    'throttle': {'ch':4,'min':1000.,'max':2000.,'rev':1.},
    'roll': {'ch':1,'min':1000.,'max':2000.,'trim':1500.},
    'pitch': {'ch':2,'min':1000.,'max':2000.,'trim':1500.},
    'yaw': {'ch':3,'min':1000.,'max':2000.,'trim':1500.},
}


class Demo9IntegrationTests(unittest.TestCase):
    def test_no_low_throttle_requirement(self):
        for value in (1020, 1300, 1500, 1750, 1980):
            ok, info = is_handoff_ready(FakeRC(value), CAL)
            self.assertTrue(ok, info)

    def test_invalid_rc_rejected(self):
        self.assertFalse(is_handoff_ready(None, CAL)[0])
        self.assertFalse(is_handoff_ready(FakeRC(4000), CAL)[0])

    def test_landing_method_substituted(self):
        method = node_method('start_land')
        source = ast.get_source_segment((ROOT/'uav2_demo5_waypoints.py').read_text(), method)
        self.assertIn("demo9_posctl_handoff.py", source)
        self.assertIn("demo9_verified_hold('FINAL_WAYPOINT', 3.0)",source)
        self.assertNotIn("'land'", source)
        self.assertNotIn('uav2_flight_ops_dev.py', source)

    def test_demo8_waypoints_modes_retained(self):
        module=ast.parse((ROOT/'uav2_demo5_waypoints.py').read_text())
        cls=next(x for x in module.body if isinstance(x,ast.ClassDef) and x.name=='TangentPreview')
        names={x.name for x in cls.body if isinstance(x,ast.FunctionDef)}
        for key in ('execute_direct','plan_execute_waypoint','execute_point_waypoint',
                    'execute_terminal_yaw','restore_point_hold','wait_airborne_hold'):
            self.assertIn(key,names)

    def test_after_takeoff_verified_hold(self):
        src=(ROOT/'uav2_demo5_waypoints.py').read_text()
        self.assertIn('self.demo9_verified_hold("AFTER_TAKEOFF", 3.0)',src)
        self.assertEqual(src.count('self.demo9_verified_hold("AFTER_TAKEOFF", 3.0)'), 1)

    def test_handoff_one_shot_posctl_only(self):
        src=(ROOT/'demo9_posctl_handoff.py').read_text()
        self.assertEqual(src.count("custom_mode='POSCTL'"), 1)
        self.assertIn('self.switch_attempted=True',src)
        self.assertNotIn("custom_mode='AUTO.LAND'", src)
        self.assertNotIn('CommandBool',src)

    def test_launcher_preserves_qr_and_yaw(self):
        self.assertIn('demo7_qr_monitor.py',(ROOT/'start_demo9.sh').read_text())
        src=(ROOT/'run_demo9.sh').read_text()
        self.assertIn('start_demo9.sh',src)
        self.assertIn('ego_bridge_status',src)
        self.assertIn('mission_event',src)
        self.assertIn('mavros/rc/in',src)
        self.assertIn('mavros/imu/data',src)

    def test_v3_takeoff_origin_untouched(self):
        # Verify that the new project copies the original V3 gate/ops bytes.
        mf=(ROOT/'v3_takeoff_origin.sha256').read_text().splitlines()
        self.assertEqual(len(mf),2)
        for row in mf:
            digest, path=row.split('  ',1)
            self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),digest)
        ops=(ROOT/'uav2_flight_ops_dev.py').read_text()
        self.assertIn('auto LAND disabled',ops)

    def test_no_ready_or_flight_entry_in_install(self):
        installer=ROOT/'run_demo9.sh'
        self.assertNotIn('V3_PILOT_READY',installer.read_text())
        self.assertNotIn('READY_CONFIRM',installer.read_text())


if __name__=='__main__':
    unittest.main(verbosity=1)
