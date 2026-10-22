#!/usr/bin/env python3
"""Offline-only regression checks for Demo5 V2 mixed POINT/EGO.
No ROS import or flight commands.
"""
from pathlib import Path
import copy
import math
import tempfile
import yaml
from demo5_config import load_config, demo_to_world

ROOT = Path(__file__).resolve().parent
CONFIG = ROOT / 'demo5_waypoints.yaml'


def load_data(data):
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / 'case.yaml'
        path.write_text(yaml.safe_dump(data))
        return load_config(path)


def rejected(data):
    try:
        load_data(data)
    except (ValueError, KeyError, TypeError):
        return
    raise AssertionError('Unsafe or invalid waypoint config accepted: %r' % data)


def run_tests():
    cfg = load_config(CONFIG)
    assert 1 <= len(cfg['waypoints']) <= 20
    assert cfg['flight_height'] == 0.8
    assert cfg['fence'] == {
        'forward_min': -2.0, 'forward_max': 8.0,
        'left_min': -8.0, 'left_max': 8.0,
        'height_min': -0.5, 'height_max': 1.5,
    }
    assert all(p['mode'] in ('POINT', 'EGO') for p in cfg['waypoints'])

    origin = (0., 0., 0.)
    new_forward = (0., 1.)
    new_left = (-1., 0.)
    for wp in cfg['waypoints']:
        got = demo_to_world(origin, new_forward, new_left,
                            wp['forward'], wp['left'], 0.8)
        target = (-wp['left'], wp['forward'], 0.8)
        assert all(abs(a-b) < 1e-9 for a, b in zip(got, target))

    fixture = copy.deepcopy(cfg)
    fixture['waypoints'] = [
        {'forward': 1., 'left': 1., 'mode': 'POINT'},
        {'forward': 3., 'left': -1., 'mode': 'EGO'},
        {'forward': 5., 'left': 0., 'mode': 'POINT'},
    ]
    normalized = load_data(fixture)
    assert [p['mode'] for p in normalized['waypoints']] == [
        'POINT', 'EGO', 'POINT'
    ]
    # Replacing a POINT leg with EGO must not affect session axes.
    fixture['waypoints'][0]['mode'] = 'EGO'
    assert load_data(fixture)['waypoints'][0]['mode'] == 'EGO'
    fixture['waypoints'][0]['mode'] = 'POINT'

    # Any count 1..20: last waypoint semantics depend on index, not mode.
    one = copy.deepcopy(fixture)
    one['waypoints'] = [{'forward': 1., 'left': 0., 'mode': 'POINT'}]
    assert len(load_data(one)['waypoints']) == 1
    twenty = copy.deepcopy(fixture)
    twenty['waypoints'] = [
        {'forward': 0.5 + 0.22*i, 'left': 0.,
         'mode': 'POINT' if i % 2 else 'EGO'}
        for i in range(20)
    ]
    assert len(load_data(twenty)['waypoints']) == 20

    bad = copy.deepcopy(fixture)
    del bad['waypoints'][0]['mode']
    rejected(bad)
    bad = copy.deepcopy(fixture)
    bad['waypoints'][0]['mode'] = 'AUTO'
    rejected(bad)
    bad = copy.deepcopy(fixture)
    bad['waypoints'][0]['mode'] = 'point'
    rejected(bad)
    bad = copy.deepcopy(fixture)
    bad['waypoints'][0]['mode'] = None
    rejected(bad)
    bad = copy.deepcopy(fixture)
    bad['waypoints'][0]['forward'] = 3.5  # >2.7m POINT leg
    rejected(bad)
    bad = copy.deepcopy(fixture)
    bad['waypoints'][0]['forward'] = float('nan')
    rejected(bad)
    bad = copy.deepcopy(fixture)
    bad['waypoints'][1]['left'] = 2.2
    rejected(bad)
    bad = copy.deepcopy(fixture)
    bad['waypoints'][1] = copy.deepcopy(bad['waypoints'][0])
    rejected(bad)
    bad = copy.deepcopy(fixture)
    bad['flight_height'] = 1.0
    rejected(bad)
    bad = copy.deepcopy(fixture)
    bad['fence']['forward_max'] = 7.
    rejected(bad)

    print('[ OK ] Demo5 V2 offline: YAML edits, 1..20 points, '
          'mixed POINT/EGO, yaw-frame transform, POINT limit, '
          'invalid configs rejected')


if __name__ == '__main__':
    run_tests()
