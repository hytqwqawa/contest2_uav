#!/usr/bin/env python3
"""End-of-waypoint handoff: verified hold -> one-shot POSCTL -> real pilot landing.

Uses no virtual RC, no LAND mode, no software ARM/DISARM and no touchdown
inference from range data below the manufacturer's rated minimum.
"""
import math
import time
from pathlib import Path

import rospy
import yaml
from mavros_msgs.msg import RCIn, State, ExtendedState, PositionTarget
from mavros_msgs.srv import ParamGet, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from rc_gate_v13 import is_handoff_ready

ROOT = Path(__file__).resolve().parent


class HandoffError(RuntimeError):
    pass


class PosctlHandoff:
    def __init__(self):
        self.data = {}
        self.stamp = {}
        self.switch_attempted = False
        self.got_posctl = False
        self.state_of = None
        inputs = (
            ('state', '/contest2/mavros/state', State),
            ('extended', '/contest2/mavros/extended_state', ExtendedState),
            ('odom', '/contest2/mavros/local_position/odom', Odometry),
            ('rc', '/contest2/mavros/rc/in', RCIn),
            ('healthy', '/contest2/ego_uav_interface/flight_healthy', Bool),
            ('mission', '/contest2/ego_uav_interface/rc_mission_allowed', Bool),
            ('safe', '/contest2/ego_uav_interface/demo_frame_safe', Bool),
            ('point', '/contest2/ego_uav_interface/point_traj_status', String),
            ('mux', '/contest2/ego_uav_interface/trajectory_mux_status', String),
            ('point_cmd', '/contest2/ego_uav_interface/point_setpoint', PositionTarget),
        )
        self.subs = [rospy.Subscriber(topic, cls, self.on_msg,
                     callback_args=key, queue_size=20) for key, topic, cls in inputs]
        self.events = rospy.Publisher('/contest2/demo9_integrated/mission_event',
                                      String, queue_size=10, latch=True)
        self.mode_srv = rospy.ServiceProxy('/contest2/mavros/set_mode', SetMode)
        self.param_srv = rospy.ServiceProxy('/contest2/mavros/param/get', ParamGet)
        session = yaml.safe_load((ROOT / 'session_frame.yaml').read_text())
        self.ground = session['origin_world']
        self.height = float(session['goal_demo']['height'])
        if not 0.5 <= self.height <= 1.2:
            raise HandoffError('Session altitude outside handoff operating envelope')
        self.cal = None

    def on_msg(self, msg, key):
        self.data[key] = msg
        self.stamp[key] = time.monotonic()

    def fresh(self, key, max_age):
        return (key in self.data and
                0 <= time.monotonic() - self.stamp.get(key, -1e12) <= max_age)

    def publish(self, message):
        print('[DEMO9 HANDOFF] ' + message, flush=True)
        self.events.publish(String(data=message))

    def px4_param(self, name, integer=False):
        ans = self.param_srv(param_id=name)
        if not ans.success:
            raise HandoffError('Unavailable RC calibration parameter ' + name)
        v = int(ans.value.integer) if integer else float(ans.value.real)
        if not integer and v == 0.0:
            v = float(ans.value.integer)
        if integer and v == 0:
            v = int(round(float(ans.value.real)))
        if not math.isfinite(v):
            raise HandoffError('Non-finite RC calibration ' + name)
        return v

    def load_calibration(self):
        self.param_srv.wait_for_service(timeout=4)
        cal = {}
        for name in ('throttle', 'roll', 'pitch', 'yaw'):
            ch = self.px4_param('RC_MAP_' + name.upper(), integer=True)
            if not 1 <= ch <= 18:
                raise HandoffError('Invalid RC_MAP_' + name.upper())
            prefix = 'RC%d_' % ch
            entry = {
                'ch': ch,
                'min': self.px4_param(prefix + 'MIN'),
                'max': self.px4_param(prefix + 'MAX'),
            }
            if name == 'throttle':
                entry['rev'] = self.px4_param(prefix + 'REV')
            else:
                entry['trim'] = self.px4_param(prefix + 'TRIM')
            cal[name] = entry
        if len({v['ch'] for v in cal.values()}) != 4:
            raise HandoffError('RC stick channel mapping overlaps')
        self.cal = cal

    def require_airborne_hold(self):
        # Retain the existing Demo8 CH5-CH8/health/position gates; only remove
        # V12's extra requirement that throttle be strictly at minimum.
        s = self.data.get('state')
        if not self.fresh('state', 2.5) or s is None or not s.connected:
            raise HandoffError('PX4 state lost')
        if not s.armed or s.mode != 'OFFBOARD':
            raise HandoffError('Expected ARMED/OFFBOARD before handoff')
        for key in ('healthy', 'mission', 'safe'):
            if not self.fresh(key, 1.0) or self.data[key].data is not True:
                raise HandoffError('Health/mission/fence permission invalid: ' + key)
        if not self.fresh('point', 1.0) or self.data['point'].data != 'AIRBORNE_HOLD':
            raise HandoffError('POINT no longer holding')
        if not self.fresh('mux', 1.0) or self.data['mux'].data != 'POINT_ACTIVE':
            raise HandoffError('MUX not POINT_ACTIVE')
        if not self.fresh('odom', 0.40) or not self.fresh('point_cmd', 0.40):
            raise HandoffError('Odometry/POINT command stale')
        pose = self.data['odom'].pose.pose.position
        vel = self.data['odom'].twist.twist.linear
        cmd = self.data['point_cmd'].position
        nums = (pose.x, pose.y, pose.z, vel.x, vel.y, vel.z, cmd.x, cmd.y, cmd.z)
        if not all(math.isfinite(float(v)) for v in nums):
            raise HandoffError('Non-finite flight estimates')
        if not self.fresh('rc', 0.30):
            raise HandoffError('RC link stale')
        valid, info = is_handoff_ready(self.data['rc'], self.cal)
        if not valid:
            raise HandoffError('RC link/calibration invalid: ' + info)
        return info

    def observe_until_disarmed(self):
        last = 0
        confirmed = False
        while not rospy.is_shutdown():
            if not self.fresh('state', 2.5):
                raise HandoffError('PX4 state stale during pilot landing')
            s = self.data['state']
            if s.mode == 'POSCTL' and s.armed and not confirmed:
                confirmed = True
                self.publish('PX4_POSCTL_CONFIRMED; pilot controls real throttle to descend')
            if not s.armed:
                ext = self.data.get('extended')
                landed = (self.fresh('extended', 2.0) and ext is not None and
                          ext.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)
                if not landed:
                    self.publish('DISARMED without fresh ON_GROUND; pilot verify physical contact')
                self.publish('DISARMED; landing observation ended')
                return
            if time.monotonic()-last >= 2:
                print('[PILOT] PX4=%s armed=%s; use true RC throttle; CH7 LOW/CH6 POSCTL takeover' %
                      (s.mode, s.armed), flush=True)
                last=time.monotonic()
            rospy.sleep(.10)
        raise HandoffError('ROS shutdown during landing')

    def run(self):
        until=time.monotonic()+8
        while not rospy.is_shutdown() and time.monotonic()<until:
            if all(self.fresh(k, .6 if k!='state' else 2.5) for k in
                   ('state','odom','rc','healthy','mission','safe','point','mux','point_cmd')):
                break
            rospy.sleep(.05)
        try:
            self.load_calibration()
            self.mode_srv.wait_for_service(timeout=3)
            info = self.require_airborne_hold()
            self.publish('P4 reached; one-shot POSCTL')
            self.publish('Real RC: ' + info)
            self.publish('POSCTL uses existing physical throttle position')
            if self.switch_attempted:
                raise HandoffError('Duplicate POSCTL request blocked')
            self.switch_attempted=True
            result=self.mode_srv(base_mode=0, custom_mode='POSCTL')
            if not result.mode_sent:
                raise HandoffError('POSCTL mode request refused by MAVROS')
            limit=time.monotonic()+3.0
            while not rospy.is_shutdown() and time.monotonic()<limit:
                if self.fresh('state', 2.5):
                    st=self.data['state']
                    if not st.armed:
                        raise HandoffError('Unexpected disarm during mode switch')
                    if st.mode=='POSCTL':
                        break
                    if st.mode!='OFFBOARD':
                        raise HandoffError('Unexpected mode '+st.mode)
                rospy.sleep(.05)
            else:
                raise HandoffError('POSCTL not confirmed after single request')
            self.observe_until_disarmed()
            return 0
        except Exception as err:
            self.publish('PILOT TAKEOVER: '+str(err))
            self.publish('CH7 LOW; CH6 POSCTL; pilot land; CH5 physical DISARM after contact')
            try:
                self.observe_until_disarmed()
            except Exception as e:
                self.publish('Pilot observation unavailable: '+str(e))
            return 2


def main():
    rospy.init_node('demo9_integrated_posctl_handoff', anonymous=False)
    return PosctlHandoff().run()


if __name__=='__main__':
    raise SystemExit(main())
