#!/usr/bin/env python3
"""Pure offline execution checks for Demo9 hold added to cloned Demo8 class."""
import ast
import math
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as O
from unittest.mock import patch

SRC=Path(__file__).with_name('uav2_demo5_waypoints.py')
parse=ast.parse(SRC.read_text())
cls=next(a for a in parse.body if isinstance(a,ast.ClassDef) and a.name=='TangentPreview')
method=next(a for a in cls.body if isinstance(a,ast.FunctionDef) and a.name=='demo9_verified_hold')
mod=ast.Module(body=[method],type_ignores=[])
ast.fix_missing_locations(mod)

class Clock:
    def __init__(self):self.t=100.
    def mono(self):return self.t
    def sleep(self,s):self.t+=s

class Stamp:
    def __init__(self,v):self.v=v
    def __sub__(self,other):return O(to_sec=lambda:self.v-other.v)

class FakeROS:
    def __init__(self,obj,clock,update_stamps=True):
        self.obj=obj;self.clock=clock;self.update_stamps=update_stamps
        self.Time=O(now=lambda:Stamp(self.clock.t))
    def is_shutdown(self):return False
    def loginfo(self,*args):pass
    def logwarn(self,*args):pass
    def sleep(self,s):
        self.clock.sleep(s)
        if self.update_stamps:
            self.obj.odom_time=Stamp(self.clock.t)
            self.obj.point_cmd_time=Stamp(self.clock.t)

class HoldTests(unittest.TestCase):
    def setup_trial(self):
        obj=O()
        obj.clock=Clock()
        obj.odom=O(pose=O(pose=O(position=O(x=0,y=0,z=.8))),
                   twist=O(twist=O(linear=O(x=0,y=0,z=0))))
        obj.state=O(connected=True,armed=True,mode='OFFBOARD')
        obj.point_cmd=O(position=O(x=0,y=0,z=.8))
        obj.oz=0.
        obj.demo5_cfg={'flight_height':.8}
        obj.flight_healthy=True;obj.mission_allowed=True;obj.demo_safe=True
        obj.point_status='AIRBORNE_HOLD';obj.mux_status='POINT_ACTIVE'
        obj.odom_time=Stamp(obj.clock.t);obj.point_cmd_time=Stamp(obj.clock.t)
        return obj
    def invoke(self,o,update_stamps=True):
        ros=FakeROS(o,o.clock,update_stamps=update_stamps)
        ns={'rospy':ros,'time':time,'math':math,'RuntimeError':RuntimeError}
        exec(compile(mod,str(SRC),'exec'),ns)
        with patch.object(time,'monotonic',o.clock.mono):
            ns['demo9_verified_hold'](o,'AFTER_TAKEOFF',3.0)
    def test_three_seconds_actual_hold(self):
        o=self.setup_trial();start=o.clock.t
        self.invoke(o)
        self.assertGreaterEqual(o.clock.t-start,3.0)
        self.assertLess(o.clock.t-start,3.2)
    def test_gates_block(self):
        for attr,val in [('flight_healthy',False),('mission_allowed',False),
                         ('demo_safe',False),('point_status','EGO_ACTIVE'),
                         ('mux_status','EGO_ACTIVE')]:
            with self.subTest(attr=attr):
                o=self.setup_trial();setattr(o,attr,val)
                with self.assertRaisesRegex(RuntimeError,'HOLD/health/RC'):
                    self.invoke(o)
    def test_height_rejection(self):
        o=self.setup_trial();o.odom.pose.pose.position.z=.5
        with self.assertRaisesRegex(RuntimeError,'hover height/VZ/XY'):
            self.invoke(o)
    def test_setpoint_discrepancy(self):
        o=self.setup_trial();o.point_cmd.position.x=.5
        with self.assertRaisesRegex(RuntimeError,'hover height/VZ/XY'):
            self.invoke(o)
    def test_missing_stream_rejected(self):
        o=self.setup_trial()
        with self.assertRaisesRegex(RuntimeError,'stale'):
            self.invoke(o,update_stamps=False)

if __name__=='__main__':unittest.main(verbosity=1)
