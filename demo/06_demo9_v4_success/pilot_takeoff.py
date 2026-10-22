#!/usr/bin/env python3
"""Physical-pilot authorization -> POINT takeoff -> measured altitude hold.

The PX4, RC supervisor, POINT controller and offboard manager keep final
control of arming, mode permission, fresh odometry and setpoint integrity.
This module does NOT arm/disarm, send MAVROS PositionTarget, or land.
"""
import time

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

BASE = '/contest2/ego_uav_interface/'


class PilotTakeoff:
    def __init__(self):
        self.values = {}
        self.times = {}
        self.ground_z = None
        topics = (
            ('state', '/contest2/mavros/state', State),
            ('odom', '/contest2/mavros/local_position/odom', Odometry),
            ('health', BASE + 'flight_healthy', Bool),
            ('stream', BASE + 'offboard_stream_ready', Bool),
            ('permission', BASE + 'rc_mission_allowed', Bool),
            ('point', BASE + 'point_traj_status', String),
            ('mux', BASE + 'trajectory_mux_status', String),
        )
        for key, topic, msg_type in topics:
            rospy.Subscriber(topic, msg_type, self.on_msg, callback_args=key, queue_size=10)
        self.services = {
            name: rospy.ServiceProxy(BASE + name, Trigger)
            for name in ('disable_trajectory', 'abort_trajectory',
                         'use_point', 'request_offboard', 'start_takeoff')
        }

    def on_msg(self, msg, key):
        self.values[key] = msg
        self.times[key] = time.monotonic()

    def get(self, key, max_age=2.0):
        age = time.monotonic() - self.times.get(key, -1e9)
        return self.values.get(key) if 0 <= age < max_age else None

    def state(self):
        return self.get('state', 2.5)

    def check_pilot(self):
        s, rc = self.state(), self.get('permission', 1.5)
        return bool(s is not None and s.connected and s.armed and
                    s.mode in ('POSCTL', 'OFFBOARD') and
                    rc is not None and rc.data)

    def status(self):
        s = self.state()
        text = ('mode=%s armed=%s' % (s.mode, s.armed)) if s else 'PX4 state stale'
        for key in ('health', 'stream', 'permission', 'point', 'mux'):
            m = self.get(key, 1.5)
            text += ' %s=%s' % (key, str(getattr(m, 'data', '?')))
        return text

    def service(self, name, timeout=3.0):
        rospy.wait_for_service(BASE + name, timeout=timeout)
        ans = self.services[name]()
        rospy.loginfo('[TAKEOFF] %s -> %s: %s', name, ans.success, ans.message)
        return ans.success, ans.message

    def wait_ground_pilot(self):
        deadline = time.monotonic() + 30
        while not rospy.is_shutdown():
            s, o, rc = self.state(), self.get('odom', 1.0), self.get('permission', 1.5)
            if s is not None and o is not None and rc is not None:
                if not s.connected or s.armed or s.mode != 'POSCTL' or rc.data:
                    raise RuntimeError('Start Terminal 3 DISARMED, POSCTL, CH7 LOW')
                self.ground_z = o.pose.pose.position.z
                rospy.set_param('/contest2/uav2_touchdown/ground_z_reference',
                                float(self.ground_z))
                break
            if time.monotonic() > deadline:
                raise RuntimeError('Missing ground MAVROS/RC messages')
            rospy.sleep(0.1)
        print('[PILOT] Ground captured; CH7 LOW -> MID -> HIGH, CH6 POSCTL, physical CH5 ARM.')
        last = 0
        while not rospy.is_shutdown():
            if self.check_pilot():
                print('[PILOT] Physical ARM and CH7 permission received.')
                return
            if time.monotonic() - last >= 2:
                print('[PILOT] Waiting: ' + self.status(), flush=True)
                last = time.monotonic()
            rospy.sleep(0.1)
        raise RuntimeError('ROS stopped while awaiting pilot')

    def prepare_point(self):
        # Each service has a single responsible node; do not duplicate its
        # validation logic here. Downstream mux checks continuity.
        self.service('disable_trajectory')
        # Wait for the actual POINT controller to accept the HOLD reset.
        for _ in range(12):
            accepted, _ = self.service('abort_trajectory')
            if accepted:
                break
            if not self.check_pilot():
                raise RuntimeError('Pilot authorization revoked before POINT reset')
            rospy.sleep(0.2)
        else:
            raise RuntimeError('POINT refused to capture a fresh ground HOLD')
        deadline = time.monotonic() + 8
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            accepted, _ = self.service('use_point')
            if accepted and (self.get('mux') is not None and
                             self.get('mux').data == 'POINT_ACTIVE'):
                return
            if not self.check_pilot():
                raise RuntimeError('Pilot authorization revoked during preparation')
            rospy.sleep(0.25)
        raise RuntimeError('POINT source unavailable; cannot issue takeoff')

    def request_offboard(self):
        print('[TAKEOFF] Waiting for existing controller readiness (no duplicate hard deadline).')
        ready_since = None
        last = 0
        while not rospy.is_shutdown():
            s = self.state()
            if not self.check_pilot():
                raise RuntimeError('Pilot authorization or PX4 link lost')
            if s.mode == 'OFFBOARD':
                return
            ready = (self.get('health') is not None and self.get('health').data
                     and self.get('stream') is not None and self.get('stream').data
                     and self.get('point') is not None and
                     self.get('point').data in ('HOLD_GROUND', 'LANDED_HOLD')
                     and self.get('mux') is not None and
                     self.get('mux').data == 'POINT_ACTIVE')
            if not ready:
                ready_since = None
            elif ready_since is None:
                ready_since = time.monotonic()
            elif time.monotonic() - ready_since >= 0.2:
                accepted, reason = self.service('request_offboard')
                # OFFBOARD callback is authoritative; transient false readiness
                # is not a reason to end the entire mission before takeoff.
                if not accepted:
                    rospy.logwarn_throttle(2.0, 'OFFBOARD pending: %s', reason)
                rospy.sleep(0.4)
            if time.monotonic() - last > 3:
                print('[TAKEOFF] ' + self.status(), flush=True)
                last = time.monotonic()
            rospy.sleep(0.15)
        raise RuntimeError('ROS stopped while awaiting OFFBOARD')

    def climb(self):
        # Bounded ACK retry for the point-controller OFFBOARD transition race.
        accepted = False
        for attempt in range(6):
            s = self.state()
            if s is None or not s.armed or s.mode != 'OFFBOARD' or not self.check_pilot():
                raise RuntimeError('Lost ARMED/OFFBOARD/pilot authorization before takeoff')
            accepted, reason = self.service('start_takeoff')
            if accepted:
                break
            if 'OFFBOARD' not in reason:
                raise RuntimeError('POINT takeoff refused: ' + reason)
            rospy.sleep(0.25)
        if not accepted:
            raise RuntimeError('POINT takeoff never accepted')
        target = self.ground_z + 0.8
        stable = None
        deadline = time.monotonic() + 16
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            s, odom, point = self.state(), self.get('odom', 1), self.get('point', 1)
            if s is None or not s.armed or s.mode != 'OFFBOARD' or not self.check_pilot():
                raise RuntimeError('Pilot/PX4 changed mode during climb; take over manually')
            if odom is not None and point is not None:
                err = abs(odom.pose.pose.position.z - target)
                ok = point.data == 'AIRBORNE_HOLD' and err <= 0.20
                if ok and stable is None:
                    stable = time.monotonic()
                if not ok:
                    stable = None
                if stable is not None and time.monotonic() - stable > 0.40:
                    print('[TAKEOFF] AIRBORNE_HOLD confirmed, altitude error %.3f m.' % err)
                    return
            rospy.sleep(0.05)
        raise RuntimeError('Takeoff was not confirmed; pilot takeover required')

    def run(self):
        self.wait_ground_pilot()
        self.prepare_point()
        self.request_offboard()
        self.climb()


if __name__ == '__main__':
    rospy.init_node('demo10_pilot_takeoff')
    try:
        PilotTakeoff().run()
    except Exception as exc:
        rospy.logerr('[TAKEOFF STOP] %s. If airborne: CH7 LOW / CH6 POSCTL; do not software disarm.', exc)
        raise SystemExit(1)
