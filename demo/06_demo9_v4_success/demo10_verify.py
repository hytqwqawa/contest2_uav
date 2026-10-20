#!/usr/bin/env python3
"""Demo10 static and ground wiring guard. NEVER ARM or switch flight modes."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parent
MANIFEST = ROOT/'DEMO10_SOURCE_MANIFEST.json'
ROUTE = [
    {'forward':3.0,'left':0.0,'mode':'EGO','yaw_deg':0.0},
    {'forward':3.0,'left':-1.0,'mode':'EGO','yaw_deg':-90.0},
    {'forward':2.0,'left':-1.5,'mode':'EGO','yaw_deg':-180.0},
    {'forward':1.0,'left':-1.5,'mode':'POINT'},
]

def fail(s):
    raise SystemExit('[STOP] Demo10 verification: '+s)

def sha(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()

def static_check(quiet=False):
    m = json.loads(MANIFEST.read_text())
    for rel, expected in m['source_sha256'].items():
        p = ROOT/rel
        if not p.is_file() or p.is_symlink() or sha(p) != expected:
            fail('Missing/modified frozen source: '+rel)
    if yaml.safe_load((ROOT/'demo5_waypoints.yaml').read_text()).get('waypoints') != ROUTE:
        fail('Four-waypoint plan changed')
    if not quiet:
        print('[PASS] Demo10 frozen four-waypoint code/config + %d SHA256 files' % len(m['source_sha256']))

def mux_check():
    expected = str(ROOT/'ros_src/ego_uav_interface/scripts/trajectory_mux_v4_only.py')
    hits = []
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        try:
            argv = (p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        except (FileNotFoundError,PermissionError,ProcessLookupError):
            continue
        if 'trajectory_mux_v4_only.py' in argv or '/scripts/trajectory_mux.py' in argv:
            # Reject any other vehicle MUX process, not just a ROS node with a matching name.
            if ('python' in argv and ('ego_uav_interface' in argv or expected in argv)):
                hits.append((p.name,argv[:220]))
    if len(hits) != 1 or expected not in hits[0][1]:
        fail('Need exactly ONE live DEMO10 MUX, no older MUX. Found '+repr(hits[:5]))
    print('[PASS] Exactly one live DEMO10 MUX, pid '+hits[0][0])

def ground_check():
    try:
        import rospy
        from mavros_msgs.msg import State
        from std_msgs.msg import Bool, String
    except ImportError as e:
        fail('ROS Noetic environment not sourced: '+str(e))
    rospy.init_node('demo10_safe_ground_check', anonymous=True, disable_signals=True)
    def msg(topic, typ):
        try:
            return rospy.wait_for_message(topic, typ, timeout=5)
        except Exception as e:
            fail('Missing ROS topic '+topic+': '+str(e))
    state = msg('/contest2/mavros/state', State)
    rc = msg('/contest2/ego_uav_interface/rc_mission_allowed', Bool)
    healthy = msg('/contest2/ego_uav_interface/flight_healthy', Bool)
    mux = msg('/contest2/ego_uav_interface/trajectory_mux_status', String)
    interface = msg('/contest2/ego_uav_interface/trajectory_interface_status', String)
    if not state.connected or state.armed or state.mode != 'POSCTL':
        fail('PX4 must be CONNECTED + DISARMED + POSCTL')
    if rc.data or not healthy.data:
        fail('CH7 mission permission not LOW or flight health not True')
    if mux.data != 'POINT_ACTIVE' or interface.data != 'HOLD_RC_BLOCKED':
        fail('Unexpected MUX/interface: %r / %r' % (mux.data,interface.data))
    print('[PASS] Ground PX4 POSCTL DISARM, CH7 LOW, health True, POINT_ACTIVE / HOLD_RC_BLOCKED')

if __name__ == '__main__':
    a = argparse.ArgumentParser()
    a.add_argument('--quiet', action='store_true')
    a.add_argument('--mux-only', action='store_true')
    a.add_argument('--ground', action='store_true')
    args = a.parse_args()
    static_check(args.quiet)
    if args.mux_only or args.ground:
        mux_check()
    if args.ground:
        ground_check()
