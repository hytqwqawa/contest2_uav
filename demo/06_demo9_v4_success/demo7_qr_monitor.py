#!/usr/bin/env python3
"""Demo7: QR text + estimated QR-center position in PX4 local (T265-aided) world.

Observer ONLY: no flight commands/services. QR text logic and flight gating unchanged.
World coordinate is published only when aligned depth, intrinsics, PX4 odom and
body<-D435 optical extrinsic are present and sufficiently time-consistent.

IMPORTANT: provided body<-camera extrinsic describes depth optical origin. The
color optical center is treated as coincident with it. Calibrate color-to-depth
extrinsics for higher accuracy; never use these positions for flight control
without independent validation.
"""
import json
import math
import threading
import time
from collections import deque

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from pyzbar.pyzbar import ZBarSymbol, decode
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from qr_stability import QrStability

NS = '/contest2'
QR_TOPIC = NS + '/demo7/qr/result'
STATUS_TOPIC = NS + '/demo7/qr/status'
CENTER_WORLD_TOPIC = NS + '/demo7/qr/center_world'
CENTER_INFO_TOPIC = NS + '/demo7/qr/center_info'
YELLOW = '\033[1;33m'
RESET = '\033[0m'


def _stamp(message):
    """Return ROS message timestamp in seconds, or None for unset stamps."""
    sec = message.header.stamp.to_sec()
    return sec if math.isfinite(sec) and sec > 0.0 else None


def _rotate(q, v):
    """Rotate 3-vector v using quaternion in ROS xyzw order."""
    quat = np.asarray([q.x, q.y, q.z, q.w], dtype=np.float64)
    n = np.linalg.norm(quat)
    if not np.isfinite(n) or n < 1e-8:
        raise ValueError('invalid quaternion')
    quat /= n
    vec = np.asarray(v, dtype=np.float64)
    t = 2.0 * np.cross(quat[:3], vec)
    return vec + quat[3] * t + np.cross(quat[:3], t)


def _center_pixel(obj):
    """Approximate visible QR center from polygon corners; rect fallback."""
    points = getattr(obj, 'polygon', None) or []
    if len(points) >= 4:
        # Use all four corners when pyzbar provides the quadrilateral.
        points = points[:4]
        return (float(sum(p.x for p in points)) / len(points),
                float(sum(p.y for p in points)) / len(points))
    r = obj.rect
    return float(r.left + r.width / 2.0), float(r.top + r.height / 2.0)


class QRMonitor:
    def __init__(self):
        self.rate = float(rospy.get_param('~decode_hz', 5.0))
        self.required = int(rospy.get_param('~stable_frames', 3))
        self.ground_test = bool(rospy.get_param('~ground_test', False))
        if not (0.5 <= self.rate <= 5.0 and 3 <= self.required <= 10):
            raise ValueError('QR rate or stability outside permitted range')

        # These gates affect QR POSITION only; QR TEXT keeps working if depth
        # or another coordinate input is absent.
        self.max_skew_s = float(rospy.get_param('~max_sensor_skew_s', 0.12))
        self.max_extrinsic_age_s = float(rospy.get_param('~max_extrinsic_age_s', 2.0))
        self.min_depth_m = float(rospy.get_param('~min_depth_m', 0.20))
        self.max_depth_m = float(rospy.get_param('~max_depth_m', 5.0))
        self.patch_radius = int(rospy.get_param('~depth_patch_radius', 4))
        if not (0.01 < self.max_skew_s <= 0.5 and
                0.1 < self.max_extrinsic_age_s <= 10.0 and
                0.0 < self.min_depth_m < self.max_depth_m and
                1 <= self.patch_radius <= 12):
            raise ValueError('invalid QR geometry parameters')

        self.bridge = CvBridge()
        self.qr = QrStability(min_frames=self.required, freshness_s=1.5)
        self.lock = threading.Lock()
        self.last_process = 0.0
        self.px4_armed = False
        self.px4_offboard = False
        self.px4_last_seen = None
        self.camera_info = None
        self.depth_history = deque(maxlen=10)
        self.odom_history = deque(maxlen=100)
        self.extrinsic = None
        self.extrinsic_seen = None
        self.last_geometry = None

        self.result_pub = rospy.Publisher(QR_TOPIC, String, queue_size=1, latch=False)
        self.status_pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=1, latch=False)
        self.center_world_pub = rospy.Publisher(
            CENTER_WORLD_TOPIC, PointStamped, queue_size=1, latch=False)
        self.center_info_pub = rospy.Publisher(
            CENTER_INFO_TOPIC, String, queue_size=1, latch=False)

        # Keep the original RGB QR input unchanged (<= 5Hz decoding).
        self.image_sub = rospy.Subscriber(
            NS + '/camera/color/image_raw', Image, self.on_image,
            queue_size=1, buff_size=4 * 1024 * 1024)
        self.px4_sub = rospy.Subscriber(
            NS + '/mavros/state', State, self.on_state, queue_size=1)
        self.depth_sub = rospy.Subscriber(
            NS + '/camera/aligned_depth_to_color/image_raw', Image,
            self.on_depth, queue_size=2, buff_size=4 * 1024 * 1024)
        self.info_sub = rospy.Subscriber(
            NS + '/camera/color/camera_info', CameraInfo,
            self.on_info, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            NS + '/mavros/local_position/odom', Odometry,
            self.on_odom, queue_size=10)
        self.extrinsic_sub = rospy.Subscriber(
            NS + '/vins_fusion/extrinsic', Odometry,
            self.on_extrinsic, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.on_second)
        rospy.loginfo('[DEMO7 QR] RGB <=%.1fHz, stable=%d frames; world position enabled, ground_test=%s',
                      self.rate, self.required, self.ground_test)

    def on_state(self, msg):
        with self.lock:
            self.px4_armed = bool(msg.connected and msg.armed)
            self.px4_offboard = (msg.mode == 'OFFBOARD')
            self.px4_last_seen = time.monotonic()

    def on_depth(self, msg):
        if _stamp(msg) is not None:
            with self.lock:
                self.depth_history.append(msg)

    def on_info(self, msg):
        if (msg.width > 0 and msg.height > 0 and
                math.isfinite(msg.K[0]) and msg.K[0] > 0 and
                math.isfinite(msg.K[4]) and msg.K[4] > 0):
            with self.lock:
                self.camera_info = msg

    def on_odom(self, msg):
        if _stamp(msg) is not None:
            with self.lock:
                self.odom_history.append(msg)

    def on_extrinsic(self, msg):
        if _stamp(msg) is not None:
            with self.lock:
                self.extrinsic = msg
                self.extrinsic_seen = time.monotonic()

    def _active_locked(self, now):
        fresh_px4 = (self.px4_last_seen is not None and
                     now - self.px4_last_seen <= 2.0)
        return self.ground_test or (fresh_px4 and self.px4_armed and self.px4_offboard)

    def _geometry(self, rgb_msg, obj):
        """Return (PointStamped, JSON-ready dict), or None if inputs unsafe."""
        t = _stamp(rgb_msg)
        if t is None:
            return None
        with self.lock:
            info = self.camera_info
            depths = tuple(self.depth_history)
            odoms = tuple(self.odom_history)
            ext = self.extrinsic
            ext_seen = self.extrinsic_seen
        if (info is None or ext is None or ext_seen is None or not depths or not odoms or
                time.monotonic() - ext_seen > self.max_extrinsic_age_s):
            return None
        if (info.width != rgb_msg.width or info.height != rgb_msg.height):
            return None

        depth_msg = min(depths, key=lambda m: abs(_stamp(m) - t))
        odom_msg = min(odoms, key=lambda m: abs(_stamp(m) - t))
        if (abs(_stamp(depth_msg) - t) > self.max_skew_s or
                abs(_stamp(odom_msg) - t) > self.max_skew_s):
            return None
        if (depth_msg.width != rgb_msg.width or depth_msg.height != rgb_msg.height or
                not odom_msg.header.frame_id):
            return None
        if depth_msg.encoding not in ('16UC1', '32FC1'):
            rospy.logwarn_throttle(10.0, '[DEMO7 QR] unsupported aligned depth encoding: %s',
                                   depth_msg.encoding)
            return None

        u, v = _center_pixel(obj)
        if not (math.isfinite(u) and math.isfinite(v)):
            return None
        x, y = int(round(u)), int(round(v))
        rad = self.patch_radius
        x0, x1 = max(0, x - rad), min(depth_msg.width, x + rad + 1)
        y0, y1 = max(0, y - rad), min(depth_msg.height, y + rad + 1)
        if x0 >= x1 or y0 >= y1:
            return None
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        patch = np.asarray(depth[y0:y1, x0:x1], dtype=np.float64)
        if depth_msg.encoding == '16UC1':
            patch *= 0.001  # RealSense 16UC1 depth is millimeters.
        valid = patch[np.isfinite(patch) & (patch >= self.min_depth_m) &
                      (patch <= self.max_depth_m)]
        if valid.size < 5:
            return None
        z = float(np.median(valid))

        fx, fy = float(info.K[0]), float(info.K[4])
        cx, cy = float(info.K[2]), float(info.K[5])
        p_camera = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=np.float64)
        ep = ext.pose.pose.position
        bp = odom_msg.pose.pose.position
        p_body = np.array([ep.x, ep.y, ep.z], dtype=np.float64) + _rotate(
            ext.pose.pose.orientation, p_camera)
        p_world = np.array([bp.x, bp.y, bp.z], dtype=np.float64) + _rotate(
            odom_msg.pose.pose.orientation, p_body)
        if not np.all(np.isfinite(p_world)):
            return None

        point = PointStamped()
        point.header.stamp = rgb_msg.header.stamp
        point.header.frame_id = odom_msg.header.frame_id  # PX4-local world, T265-aided
        point.point.x, point.point.y, point.point.z = map(float, p_world)
        data = {
            'stamp': round(t, 6),
            'frame_id': point.header.frame_id,
            'pixel_xy': [round(u, 2), round(v, 2)],
            'depth_z_m': round(z, 4),
            'camera_xyz_m': [round(float(a), 4) for a in p_camera],
            'world_xyz_m': [round(float(a), 4) for a in p_world],
            'rgb_depth_skew_s': round(abs(_stamp(depth_msg) - t), 4),
            'rgb_odom_skew_s': round(abs(_stamp(odom_msg) - t), 4),
        }
        return point, data

    def on_image(self, msg):
        now = time.monotonic()
        with self.lock:
            if now - self.last_process < 1.0 / self.rate:
                return
            self.last_process = now
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            objects = decode(rgb, symbols=[ZBarSymbol.QRCODE])
            values = []
            matched = []
            for obj in objects:
                try:
                    code = obj.data.decode('utf-8', errors='strict')
                except UnicodeError:
                    continue
                if code and len(code) <= 512:
                    values.append(code)
                    matched.append((code, obj))
            with self.lock:
                self.qr.process(values, time.monotonic())
                visible = self.qr.current(time.monotonic())
                active = self._active_locked(time.monotonic())
            # If duplicate physical QR codes share the same payload, their
            # coordinates are ambiguous even though the original text is stable.
            matches = [obj for code, obj in matched if code == visible]
        except Exception as exc:
            rospy.logerr_throttle(5.0, '[DEMO7 QR] QR decoding error: %s', str(exc))
            with self.lock:
                self.qr.process([], time.monotonic())
            return
        if not active or not visible or len(matches) != 1 or len(matched) != 1:
            return
        try:
            result = self._geometry(msg, matches[0])
            if result is None:
                return  # Never publish a stale or fabricated position.
            point, info = result
            info['content'] = visible
            self.center_world_pub.publish(point)
            self.center_info_pub.publish(String(data=json.dumps(info, ensure_ascii=False)))
            with self.lock:
                self.last_geometry = (visible, time.monotonic(), info)
        except Exception as exc:
            # Geometry failures MUST NOT clear the successful QR text detection.
            rospy.logwarn_throttle(5.0, '[DEMO7 QR] geometry unavailable: %s', str(exc))

    def on_second(self, _event):
        now = time.monotonic()
        with self.lock:
            active = self._active_locked(now)
            visible = self.qr.current(now)
            state = self.qr.status(now)
            geometry = self.last_geometry
        if not active:
            self.result_pub.publish(String(data=''))
            self.status_pub.publish(String(data='IDLE'))
            return
        self.result_pub.publish(String(data=visible))
        self.status_pub.publish(String(data=state))
        if visible:
            safe = json.dumps(visible, ensure_ascii=False)
            print(YELLOW + '[DEMO7 QR] 1Hz detected: ' + safe + RESET, flush=True)
            if (geometry is not None and geometry[0] == visible and
                    now - geometry[1] <= 0.5):
                info = geometry[2]
                print(YELLOW + '[DEMO7 QR XYZ] ' +
                      json.dumps({'pixel_xy': info['pixel_xy'],
                                  'depth_z_m': info['depth_z_m'],
                                  'camera_xyz_m': info['camera_xyz_m'],
                                  'world_xyz_m': info['world_xyz_m'],
                                  'frame_id': info['frame_id'],
                                  'sample_age_s': round(now - geometry[1], 3)},
                                 ensure_ascii=False) + RESET, flush=True)


if __name__ == '__main__':
    rospy.init_node('demo7_qr_monitor', anonymous=False)
    QRMonitor()
    rospy.spin()
