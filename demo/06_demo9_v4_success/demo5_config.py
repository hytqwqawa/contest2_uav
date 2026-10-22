#!/usr/bin/env python3
"""Pure Python validator/converter for Demo5 frame, fence, and ordered EGO goals.

No ROS import: can be verified on DISARMED ground or off-aircraft.
"""
import argparse
import math
from pathlib import Path
import yaml


def finite_number(v, label):
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError('%s must be a real number' % label)
    x = float(v)
    if not math.isfinite(x):
        raise ValueError('%s must be finite' % label)
    return x


def load_config(path):
    path = Path(path)
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError('Demo5 YAML must contain a mapping')
    fence = data.get('fence')
    if not isinstance(fence, dict):
        raise ValueError('fence must be a mapping')
    keys = ('forward_min','forward_max','left_min','left_max',
            'height_min','height_max')
    fence = {k: finite_number(fence[k], 'fence.'+k) for k in keys}
    for axis in ('forward','left','height'):
        if fence[axis+'_min'] >= fence[axis+'_max']:
            raise ValueError('invalid '+axis+' fence interval')
    # This run is explicitly limited to the agreed geometry. Do not silently
    # expand it by editing just one section of the control stack.
    expected = {'forward_min':-2.0, 'forward_max':8.0,
                'left_min':-8.0, 'left_max':8.0,
                'height_min':-0.5, 'height_max':1.5}
    for k, v in expected.items():
        if abs(fence[k] - v) > 1e-9:
            raise ValueError('Demo5 fixed agreed fence %s must remain %.2f' % (k,v))
    h = finite_number(data['flight_height'], 'flight_height')
    # The reused Demo4/3 POINT takeoff and landing launch is 0.80 m.
    # Do not allow YAML to change altitude without coordinated POINT changes.
    if abs(h-0.80) > 1e-9:
        raise ValueError('flight_height must remain 0.80 m for this version')
    if not (fence['height_min'] + 0.20 < h < fence['height_max'] - 0.20):
        raise ValueError('flight_height too close to height fence')
    timeout = finite_number(data.get('segment_timeout_s',90.), 'segment_timeout_s')
    if not (30. <= timeout <= 180.):
        raise ValueError('segment_timeout_s outside [30,180]')
    margin = finite_number(data.get('waypoint_margin',0.2), 'waypoint_margin')
    if not (0.1 <= margin <= 0.5):
        raise ValueError('waypoint_margin outside [0.10,0.50]')
    points = data.get('waypoints')
    if not isinstance(points, list) or not (1 <= len(points) <= 20):
        raise ValueError('waypoints must contain 1..20 ordered goals')
    normalized = []
    previous = (0.0, 0.0)
    for index, item in enumerate(points, 1):
        if not isinstance(item, dict) or not {'forward','left','mode'} <= set(item.keys()) or not set(item.keys()) <= {'forward','left','mode','yaw_deg'}:
            raise ValueError('waypoint %d requires forward, left, mode; optional EGO yaw_deg' % index)
        f = finite_number(item['forward'], 'P%d.forward' % index)
        l = finite_number(item['left'], 'P%d.left' % index)
        mode = item['mode']
        if not isinstance(mode, str) or mode not in ('POINT', 'EGO'):
            raise ValueError('P%d.mode must be POINT or EGO (uppercase)' % index)
        yaw_deg = None
        if 'yaw_deg' in item:
            if mode != 'EGO':
                raise ValueError('P%d.yaw_deg only supported on EGO, not POINT' % index)
            yaw_deg = finite_number(item['yaw_deg'], 'P%d.yaw_deg' % index)
            if not (-180.0 <= yaw_deg <= 180.0):
                raise ValueError('P%d.yaw_deg must be within [-180, 180]' % index)
        if not (fence['forward_min']+margin <= f <= fence['forward_max']-margin):
            raise ValueError('waypoint %d forward outside fence margin' % index)
        if not (fence['left_min']+margin <= l <= fence['left_max']-margin):
            raise ValueError('waypoint %d left outside fence margin' % index)
        distance = math.hypot(f-previous[0], l-previous[1])
        if distance < 0.20:
            raise ValueError('waypoint %d too close to previous goal' % index)
        # Existing POINT node hard-rejects goals >3m from the ACTUAL pose.
        # Leave 0.30m for completion tolerance, odometry drift and handoff.
        # EGO has its separate 90s segment timeout and obstacle replanning.
        if mode == 'POINT' and distance > 2.70:
            raise ValueError(
                'P%d POINT leg %.2fm exceeds 2.70m safe config limit; '
                'choose EGO or add a nearer waypoint' % (index, distance)
            )
        normalized_item = {'forward':f,'left':l,'mode':mode}
        if yaw_deg is not None:
            normalized_item['yaw_deg'] = yaw_deg
        normalized.append(normalized_item)
        previous = (f,l)
    return {'fence':fence,'flight_height':h,
            'segment_timeout_s':timeout,'waypoint_margin':margin,
            'waypoints':normalized}


def demo_to_world(origin, forward, left, f, l, h):
    """World pose: O + F*forward_axis + L*left_axis + h*Z."""
    vals = (*origin, *forward, *left, f, l, h)
    if not all(math.isfinite(float(x)) for x in vals):
        raise ValueError('non-finite transform inputs')
    if abs(math.hypot(*forward)-1.) > 0.03 or abs(math.hypot(*left)-1.) > 0.03:
        raise ValueError('non-unit session axis')
    if abs(forward[0]*left[0]+forward[1]*left[1]) > 0.03:
        raise ValueError('non-orthogonal session axes')
    return (origin[0]+f*forward[0]+l*left[0],
            origin[1]+f*forward[1]+l*left[1], origin[2]+h)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--check', required=True, help='Path to demo5_waypoints.yaml')
    args = p.parse_args()
    cfg = load_config(args.check)
    fence = cfg['fence']
    print('[ OK ] Demo5 YAML valid, %d configurable waypoints' % len(cfg['waypoints']))
    print('F fence [%.2f,%.2f], L fence [%.2f,%.2f], H fence [%.2f,%.2f]' %
          (fence['forward_min'],fence['forward_max'],
           fence['left_min'],fence['left_max'],
           fence['height_min'],fence['height_max']))
    for i, w in enumerate(cfg['waypoints'],1):
        suffix = 'FINAL/LAND' if i == len(cfg['waypoints']) else 'CONTINUE'
        yaw_text = (' yaw_demo=%+.1f deg' % w['yaw_deg']) if 'yaw_deg' in w else ' yaw=original'
        print('P%d  F=%+.2f L=%+.2f H=%.2f  %s/%s%s' %
              (i,w['forward'],w['left'],cfg['flight_height'],w['mode'],suffix,yaw_text))

if __name__ == '__main__':
    main()
