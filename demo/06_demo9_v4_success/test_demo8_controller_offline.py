#!/usr/bin/env python3
"""No-ROS simulation of Demo8 yaw phase; tests authorization & measured ACK.

Extracts the actual runner method (not a re-implementation) and executes with
fake clocks, subscriptions and publishers. NEVER starts ROS / flight software.
"""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as Obj

from demo8_terminal_yaw import TerminalYawGate

SCRIPT = Path(__file__).with_name('uav2_demo5_waypoints.py')
module = ast.parse(SCRIPT.read_text())
runner = next(n for n in module.body if isinstance(n, ast.ClassDef) and n.name == 'TangentPreview')
method = next(n for n in runner.body if isinstance(n, ast.FunctionDef) and n.name == 'execute_terminal_yaw')
standalone = ast.Module(body=[method], type_ignores=[])
ast.fix_missing_locations(standalone)

class Clock:
    def __init__(self): self.value = 1.0
    def time(self): return self.value
    def monotonic(self): return self.value

class Duration:
    def __init__(self, v): self.value = v
    def to_sec(self): return self.value

class Stamp:
    def __init__(self, v): self.value = v
    def __sub__(self, other): return Duration(self.value - other.value)

class Message:
    def __init__(self, data=None): self.data = data

class Bus:
    def __init__(self, node=None, mode=False):
        self.published = []
        self.node = node
        self.mode = mode
    def get_num_connections(self): return 1
    def publish(self, msg):
        self.published.append(msg)
        if self.mode:
            self.node.bridge_status = 'mode=%s traj=3 vxy=0' % msg.data
            self.node.bridge_status_time = Stamp(self.node.clock.value)

class PositionTarget:
    IGNORE_YAW_RATE = 2048

class FakeRos:
    def __init__(self, clock, node):
        self.clock, self.node = clock, node
        self.Time = Obj(now=lambda: Stamp(clock.value))
    def sleep(self, sec):
        self.clock.value += sec
        self.refresh()
    def refresh(self):
        self.node.odom_time = Stamp(self.clock.value)
        self.node.cmd_time = Stamp(self.clock.value)
        self.node.ego_setpoint_time = Stamp(self.clock.value)
        self.node.if_status_time = Stamp(self.clock.value)
        self.node.mavros_sp_time = Stamp(self.clock.value)
        if self.node.bridge_status.startswith('mode=FIXED'):
            self.node.bridge_status_time = Stamp(self.clock.value)
    def Rate(self, hertz):
        return Obj(sleep=lambda: self.sleep(1.0 / hertz))
    def is_shutdown(self): return False
    def logwarn(self, *args): pass
    def loginfo_throttle(self, *args): pass


def make_trial(ch7=True, commanded_yaw=0.0):
    clock = Clock()
    n = Obj()
    n.clock = clock
    n.active_wp_num, n.wp_total = 2, 3
    n.odom = Obj(pose=Obj(pose=Obj(position=Obj(x=3.,y=.5,z=.8))),
                 twist=Obj(twist=Obj(linear=Obj(x=0.,y=0.,z=0.))))
    n.state = Obj(connected=True, armed=True, mode='OFFBOARD')
    n.flight_healthy, n.mission_allowed, n.demo_safe = True, ch7, True
    n.mux_status = 'EGO_ACTIVE'
    n.odom_time = n.cmd_time = n.ego_setpoint_time = Stamp(clock.value)
    n.ego_setpoint = Obj(
        type_mask=2048, position=Obj(x=3.,y=.5,z=.8),
        velocity=Obj(x=0.,y=0.,z=0.),
        acceleration_or_force=Obj(x=0.,y=0.,z=0.), yaw=commanded_yaw)
    n.bridge_status = 'mode=TANGENT traj=3 vxy=0'
    n.bridge_status_time = Stamp(clock.value)
    n.if_status = 'TRAJECTORY_ACTIVE'
    n.if_status_time = Stamp(clock.value)
    n.mavros_sp = n.ego_setpoint
    n.mavros_sp_time = Stamp(clock.value)
    n.bspline = None
    n.active_wp_pub = Bus()
    n.yaw_target_pub = Bus()
    n.yaw_mode_pub = Bus(n, mode=True)
    n.x_min, n.x_max, n.y_min, n.y_max, n.h_min, n.h_max = -1.,6.,-2.,3.,-.25,1.3
    n.to_demo = lambda xyz: xyz
    n.current_yaw = lambda: 0.0
    fake_ros = FakeRos(clock,n)
    ns = {'rospy': fake_ros, 'time': clock, 'math': math,
          'TerminalYawGate': TerminalYawGate, 'String': Message,
          'Float64': Message, 'PositionTarget': PositionTarget}
    exec(compile(standalone, str(SCRIPT), 'exec'), ns)
    return n, ns['execute_terminal_yaw']


def expect_fail(node, fn, match, deadline):
    try:
        fn(node, 3.,.5,.8,0.,deadline)
    except RuntimeError as exc:
        assert match in str(exc), str(exc)
        return
    raise AssertionError('expected yaw phase to reject: ' + match)

# Actual runner must achieve 1s measured stability before successful return.
n, execute = make_trial()
execute(n, 3.,.5,.8,0.,n.clock.time()+5.)
assert n.clock.value >= 2.0, 'no measured stability wait'
assert len(n.yaw_target_pub.published) == 1
assert [m.data for m in n.yaw_mode_pub.published] == ['FIXED']

# CH7 missing: abort BEFORE selecting fixed mode or publishing yaw target.
n, execute = make_trial(ch7=False)
expect_fail(n, execute, 'precheck failed', n.clock.time()+5.)
assert not n.yaw_target_pub.published and not n.yaw_mode_pub.published

# Wrong bridge yaw command: measured yaw coincidentally at target MUST NOT pass.
n, execute = make_trial(commanded_yaw=math.radians(25.))
expect_fail(n, execute, 'timeout', n.clock.time()+1.6)
# Missing interface authorization MUST block yaw before publishing target/mode.
n, execute = make_trial()
n.if_status = 'HOLD_ACTIVE'
expect_fail(n, execute, 'precheck failed', n.clock.time()+5.)
assert not n.yaw_target_pub.published and not n.yaw_mode_pub.published
print('[ OK ] Demo8 EGO FIXED mock: ACK, measured output, CH7 and interface gates')
