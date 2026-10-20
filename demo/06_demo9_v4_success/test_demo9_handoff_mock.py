#!/usr/bin/env python3
"""ROS-free behavioral tests of the actual integrated handoff implementation.

No ROS master, service, real RC, arming or flight commands are created.
"""
import importlib
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
rospy=types.ModuleType('rospy')
rospy.ROSException=type('ROSException',(RuntimeError,),{})
rospy.is_shutdown=lambda:False
rospy.sleep=lambda _:None
sys.modules['rospy']=rospy
for name in ('mavros_msgs','mavros_msgs.msg','mavros_msgs.srv',
             'nav_msgs','nav_msgs.msg','std_msgs','std_msgs.msg'):
    sys.modules[name]=types.ModuleType(name)
for name in ('RCIn','State','ExtendedState','PositionTarget'):
    setattr(sys.modules['mavros_msgs.msg'],name,type(name,(),{}))
for name in ('SetMode','ParamGet'):
    setattr(sys.modules['mavros_msgs.srv'],name,type(name,(),{}))
sys.modules['mavros_msgs.msg'].ExtendedState.LANDED_STATE_ON_GROUND=1
sys.modules['nav_msgs.msg'].Odometry=type('Odometry',(),{})
sys.modules['std_msgs.msg'].Bool=type('Bool',(),{})
sys.modules['std_msgs.msg'].String=lambda data:NS(data=data)

integration=importlib.import_module('demo9_posctl_handoff')

class Clock:
    def __init__(self):self.now=1234.
    def mono(self):return self.now
    def sleep(self,s):self.now+=s

class ModeService:
    def __init__(self,obj,requested='POSCTL',sent=True,service_delay=0):
        self.obj=obj;self.requested=requested;self.sent=sent
        self.calls=[];self.service_delay=service_delay
    def wait_for_service(self,timeout):
        self.obj.clock.now+=self.service_delay
    def __call__(self,**kw):
        self.calls.append(kw)
        if self.sent and self.requested is not None:
            self.obj.data['state'].mode=self.requested
        return NS(mode_sent=self.sent)

CAL={
  'throttle':{'ch':4,'min':1000.,'max':2000.,'rev':1.},
  'roll':{'ch':1,'min':1000.,'max':2000.,'trim':1500.},
  'pitch':{'ch':2,'min':1000.,'max':2000.,'trim':1500.},
  'yaw':{'ch':3,'min':1000.,'max':2000.,'trim':1500.},
}

class Tests(unittest.TestCase):
    def setup_obj(self,thr=1500):
        o=integration.PosctlHandoff.__new__(integration.PosctlHandoff)
        o.clock=Clock()
        o.ground={'x':0.,'y':0.,'z':0.}
        o.height=.8
        o.cal=CAL
        o.switch_attempted=False
        o.got_posctl=False
        o.data={
          'state':NS(connected=True,armed=True,mode='OFFBOARD'),
          'healthy':NS(data=True),'mission':NS(data=True),'safe':NS(data=True),
          'point':NS(data='AIRBORNE_HOLD'),'mux':NS(data='POINT_ACTIVE'),
          'point_cmd':NS(position=NS(x=0.,y=0.,z=.8)),
          'odom':NS(pose=NS(pose=NS(position=NS(x=0.,y=0.,z=.8))),
                     twist=NS(twist=NS(linear=NS(x=0.,y=0.,z=0.)))),
          'rc':NS(channels=[1500,1500,1500,thr,1000,2000,1000,1000]),
          'extended':NS(landed_state=2),
        }
        o.stamp={k:o.clock.now for k in o.data}
        o.events=[]
        o.events_pub=[]
        o.publish=lambda msg:o.events.append(msg)
        o.observe_until_disarmed=lambda:None
        o.load_calibration=lambda:None
        o.mode_srv=ModeService(o)
        return o
    def call(self,o):
        with patch.object(integration.time,'monotonic',o.clock.mono),\
             patch.object(integration.rospy,'sleep',o.clock.sleep):
            return o.run()
    def check(self,o):
        with patch.object(integration.time,'monotonic',o.clock.mono):
            return o.require_airborne_hold()

    def test_normal_handoff_nonminimum_throttle(self):
        for thr in (1000,1140,1490,1600,1950):
            with self.subTest(thr=thr):
                o=self.setup_obj(thr)
                self.assertEqual(self.call(o),0)
                self.assertEqual(o.mode_srv.calls,[{'base_mode':0,'custom_mode':'POSCTL'}])
                self.assertTrue(o.switch_attempted)
    def test_rc_stale_blocks_request(self):
        o=self.setup_obj();o.stamp['rc']-=.40
        self.assertEqual(self.call(o),2);self.assertEqual(o.mode_srv.calls,[])
    def test_bad_rc_blocks_request(self):
        o=self.setup_obj(thr=2600)
        self.assertEqual(self.call(o),2);self.assertEqual(o.mode_srv.calls,[])
    def test_health_gate(self):
        for key in ('healthy','mission','safe'):
            with self.subTest(key=key):
                o=self.setup_obj();o.data[key].data=False
                self.assertEqual(self.call(o),2);self.assertEqual(o.mode_srv.calls,[])
    def test_hold_gate(self):
        for key,value in [('point','TAKEOFF_TRAJECTORY'),('mux','EGO_ACTIVE')]:
            with self.subTest(key=key):
                o=self.setup_obj();o.data[key].data=value
                self.assertEqual(self.call(o),2);self.assertEqual(o.mode_srv.calls,[])
    def test_altitude_and_xy_gate(self):
        for z,x,vz in [(.52,0,0),(.99,0,0),(.8,.27,0),(.8,0,.3)]:
            with self.subTest(z=z,x=x,vz=vz):
                o=self.setup_obj()
                o.data['odom'].pose.pose.position.z=z
                o.data['odom'].pose.pose.position.x=x
                o.data['odom'].twist.twist.linear.z=vz
                self.assertEqual(self.call(o),2);self.assertEqual(o.mode_srv.calls,[])
    def test_mode_refusal_not_retried(self):
        o=self.setup_obj();o.mode_srv.sent=False
        self.assertEqual(self.call(o),2)
        self.assertEqual(len(o.mode_srv.calls),1)
        self.assertTrue(o.switch_attempted)
        self.assertEqual(self.call(o),2)
        self.assertEqual(len(o.mode_srv.calls),1)
    def test_mode_no_ack_not_retried(self):
        o=self.setup_obj();o.mode_srv.requested=None
        self.assertEqual(self.call(o),2)
        self.assertEqual(len(o.mode_srv.calls),1)
    def test_service_wait_rc_stale(self):
        o=self.setup_obj();o.mode_srv.service_delay=.38
        self.assertEqual(self.call(o),2)
        self.assertEqual(o.mode_srv.calls,[])
    def test_wrong_px4_mode(self):
        o=self.setup_obj();o.data['state'].mode='POSCTL'
        self.assertEqual(self.call(o),2);self.assertEqual(o.mode_srv.calls,[])
    def test_unexpected_mode(self):
        o=self.setup_obj();o.mode_srv.requested='AUTO.LAND'
        self.assertEqual(self.call(o),2);self.assertEqual(len(o.mode_srv.calls),1)
    def test_missing_odom(self):
        o=self.setup_obj();o.stamp['odom']-=.5
        self.assertEqual(self.call(o),2);self.assertEqual(o.mode_srv.calls,[])

if __name__=='__main__':unittest.main(verbosity=1)
