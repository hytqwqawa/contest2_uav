#!/usr/bin/env python3
"""Read-only fourth terminal: print stable QR/color detections at 1 Hz.

No ARM or OFFBOARD is needed IF both observer nodes run with ground_test=true.
No detections => completely silent; old coordinates are never substituted.
Expected ROS namespace is /contest1 even when the filesystem root is contest2_uav.
"""

import json
import math
import threading
import time
from datetime import datetime

import rospy
from std_msgs.msg import String

NS = '/contest2/demo7'


def coords_fmt(xyz):
    if not isinstance(xyz, (list, tuple)) or len(xyz) != 3:
        return '不可用'
    try:
        vals = [float(x) for x in xyz]
        if not all(math.isfinite(v) for v in vals):
            return '不可用'
        return '({:+.3f}, {:+.3f}, {:+.3f})m'.format(*vals)
    except (ValueError, TypeError):
        return '不可用'


def pixel_fmt(xy):
    if not isinstance(xy, (list, tuple)) or len(xy) != 2:
        return '不可用'
    try:
        vals = [float(x) for x in xy]
        return '({:.0f}, {:.0f})px'.format(*vals) if all(map(math.isfinite, vals)) else '不可用'
    except (ValueError, TypeError):
        return '不可用'


class VisionConsole:
    def __init__(self):
        self.lock = threading.Lock()
        self.qr_value = ''
        self.qr_value_seen = None
        self.qr_geometry = None
        self.qr_geometry_seen = None
        self.color_message = None
        self.color_seen = None
        rospy.Subscriber(NS + '/qr/result', String, self.on_qr_result, queue_size=1)
        rospy.Subscriber(NS + '/qr/center_info', String, self.on_qr_info, queue_size=1)
        rospy.Subscriber(NS + '/color/detections', String, self.on_colors, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.on_second)

    def on_qr_result(self, msg):
        with self.lock:
            self.qr_value = msg.data
            self.qr_value_seen = time.monotonic()
            if not msg.data:
                self.qr_geometry = None
                self.qr_geometry_seen = None

    def on_qr_info(self, msg):
        try:
            result = json.loads(msg.data)
            if not isinstance(result, dict) or not isinstance(result.get('content'), str):
                return
        except (ValueError, TypeError):
            return
        with self.lock:
            self.qr_geometry = result
            self.qr_geometry_seen = time.monotonic()

    def on_colors(self, msg):
        try:
            parsed = json.loads(msg.data)
            if (not isinstance(parsed, dict) or
                    not isinstance(parsed.get('targets'), list)):
                return
        except (ValueError, TypeError):
            return
        with self.lock:
            self.color_message = parsed
            self.color_seen = time.monotonic()

    def on_second(self, _event):
        now = time.monotonic()
        with self.lock:
            qr = (self.qr_value if self.qr_value_seen is not None and
                  now - self.qr_value_seen <= 1.6 else '')
            qinfo = (self.qr_geometry if self.qr_geometry_seen is not None and
                     now - self.qr_geometry_seen <= 0.85 else None)
            colors = (self.color_message if self.color_seen is not None and
                      now - self.color_seen <= 1.15 else None)
        lines = []
        if qr:
            geometry = qinfo if qinfo and qinfo.get('content') == qr else {}
            pixel = pixel_fmt(geometry.get('pixel_xy'))
            world = coords_fmt(geometry.get('world_xyz_m'))
            frame = geometry.get('frame_id') or '无'
            lines.append('二维码 {} | 像素 {} | 世界坐标 {} [{}]'.format(
                json.dumps(qr, ensure_ascii=False), pixel, world, frame))
        if colors:
            for item in colors['targets']:
                if not isinstance(item, dict) or item.get('color') not in ('red','green','blue'):
                    continue
                color = {'red': '红色', 'green': '绿色', 'blue': '蓝色'}[item['color']]
                lines.append('{} | 像素 {} | 世界坐标 {} [{}]'.format(
                    color, pixel_fmt(item.get('pixel_xy')),
                    coords_fmt(item.get('world_xyz_m')), item.get('frame_id') or '无'))
        # In particular, never print SEARCHING / IDLE / NONE / stale coordinates.
        if lines:
            label = datetime.now().strftime('%H:%M:%S')
            print('\n'.join('[{}] {}'.format(label, s) for s in lines), flush=True)


def main():
    rospy.init_node('demo7_vision_console', anonymous=False, log_level=rospy.WARN)
    VisionConsole()
    rospy.spin()


if __name__ == '__main__':
    main()
