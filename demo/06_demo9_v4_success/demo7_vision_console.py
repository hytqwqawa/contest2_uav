#!/usr/bin/env python3
"""Read-only fourth terminal: print stable QR/color detections at 1 Hz.

No ARM or OFFBOARD is needed IF both observer nodes run with ground_test=true.
No detections => completely silent; old coordinates are never substituted.
Expected ROS namespace is /contest1 even when the filesystem root is contest2_uav.
"""

import json
import math
import threading
import sys
import time
from datetime import datetime

import rospy
from std_msgs.msg import Bool, String

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
    def __init__(self, color_only_window=False):
        self.lock = threading.Lock()
        self.color_only_window = bool(color_only_window)
        self.window_active = not self.color_only_window
        self.last_window_print = None
        self.qr_value = ''
        self.qr_value_seen = None
        self.qr_geometry = None
        self.qr_geometry_seen = None
        self.color_message = None
        self.color_seen = None
        if not self.color_only_window:
            rospy.Subscriber(NS + '/qr/result', String, self.on_qr_result, queue_size=1)
            rospy.Subscriber(NS + '/qr/center_info', String, self.on_qr_info, queue_size=1)
        else:
            rospy.Subscriber(NS + '/color/window_active', Bool,
                             self.on_color_window, queue_size=1)
        rospy.Subscriber(NS + '/color/detections', String, self.on_colors, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.on_second)

    def on_color_window(self, msg):
        with self.lock:
            self.window_active = bool(msg.data)
            self.color_message = None
            self.color_seen = None
            self.last_window_print = None

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
            if self.color_only_window and not self.window_active:
                return
            now = time.monotonic()
            self.color_message = parsed
            self.color_seen = now
            immediate_print = (self.color_only_window and bool(parsed['targets']) and
                               (self.last_window_print is None or
                                now - self.last_window_print >= 1.0))
            if immediate_print:
                self.last_window_print = now
        # Print the first confirmed color result immediately; a 1Hz timer
        # alone can miss short arrival/yaw/preview windows.
        if immediate_print:
            self.print_color_results(parsed)

    def print_color_results(self, colors):
        lines = []
        for item in colors['targets']:
            if not isinstance(item, dict) or item.get('color') not in ('red', 'green', 'blue'):
                continue
            color = {'red': '红色', 'green': '绿色', 'blue': '蓝色'}[item['color']]
            lines.append('{} | 像素 {} | 世界坐标 {} [{}]'.format(
                color, pixel_fmt(item.get('pixel_xy')),
                coords_fmt(item.get('world_xyz_m')), item.get('frame_id') or '无'))
        if lines:
            label = datetime.now().strftime('%H:%M:%S')
            print('\n'.join('[{}] {}'.format(label, line) for line in lines), flush=True)

    def on_second(self, _event):
        if self.color_only_window:
            return  # Window console prints immediately from fresh detections.
        now = time.monotonic()
        with self.lock:
            qr = (self.qr_value if self.qr_value_seen is not None and
                  now - self.qr_value_seen <= 1.6 else '')
            qinfo = (self.qr_geometry if self.qr_geometry_seen is not None and
                     now - self.qr_geometry_seen <= 0.85 else None)
            colors = (self.color_message if self.window_active and
                      self.color_seen is not None and
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
    color_only_window = '--color-only-window' in rospy.myargv(argv=sys.argv)[1:]
    rospy.init_node('demo7_vision_console', anonymous=color_only_window,
                    log_level=rospy.WARN)
    VisionConsole(color_only_window=color_only_window)
    rospy.spin()


if __name__ == '__main__':
    main()
