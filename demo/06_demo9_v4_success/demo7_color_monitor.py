#!/usr/bin/env python3
"""Demo7: read-only RED/GREEN/BLUE observer with RGB pixel + PX4-local XYZ.

Matches the QR observer's D435 color/aligned-depth/odometry/extrinsic topics.
No flight command, arming, mode change, or UAV service is created here.
At most one unambiguous object per color is reported; colors are independent.
For ground experiments, set /contest1/demo7_color_monitor/ground_test=true BEFORE
launching this node. Coordinates are estimates, not flight-control inputs.
"""

import json
import math
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from color_stability import COLORS, ColorStability

NS = '/contest2'
TOPIC = NS + '/demo7/color/detections'


def stamp_s(msg):
    try:
        s = float(msg.header.stamp.to_sec())
        return s if math.isfinite(s) and s > 0 else None
    except (AttributeError, TypeError, ValueError):
        return None


def rotate(q, v):
    """ROS quaternion (x,y,z,w) applied to an XYZ 3-vector, as in QR monitor."""
    quat = np.asarray([q.x, q.y, q.z, q.w], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError('invalid orientation quaternion')
    quat /= norm
    vec = np.asarray(v, dtype=np.float64)
    t = 2.0 * np.cross(quat[:3], vec)
    return vec + quat[3] * t + np.cross(quat[:3], t)


def read_config(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('color_detection.yaml must be a mapping')
    for color in COLORS:
        ranges = data.get('hue_ranges', {}).get(color)
        if not isinstance(ranges, list) or not ranges:
            raise ValueError('missing hue ranges for ' + color)
        for pair in ranges:
            if (not isinstance(pair, list) or len(pair) != 2 or
                    not all(isinstance(x, int) for x in pair) or
                    not 0 <= pair[0] <= pair[1] <= 179):
                raise ValueError('invalid HSV range for ' + color)
    for key in ('morph_open_size', 'morph_close_size'):
        v = int(data[key])
        if v < 1 or v > 19 or v % 2 != 1:
            raise ValueError(key + ' must be an odd number from 1 to 19')
    if not (0.5 <= float(data['decode_hz']) <= 5.0 and
            3 <= int(data['stable_frames']) <= 10 and
            0 < float(data['freshness_s']) <= 3 and
            5 <= float(data['max_centroid_jump_px']) <= 500 and
            0 < float(data['min_contour_area_px']) and
            0 < float(data['max_contour_area_fraction']) <= 0.95 and
            0 <= int(data['saturation_min']) <= 255 and
            0 <= int(data['value_min']) <= 255 and
            0.01 < float(data['max_sensor_skew_s']) <= 0.5 and
            0.1 <= float(data['max_extrinsic_age_s']) <= 10 and
            0 < float(data['min_depth_m']) < float(data['max_depth_m']) and
            0 <= int(data['depth_interior_erode_px']) <= 12 and
            5 <= int(data['min_valid_depth_pixels']) <= 10000):
        raise ValueError('invalid color detection parameters')
    return data


class ColorMonitor:
    def __init__(self):
        path = rospy.get_param('~config', str(Path(__file__).with_name('color_detection.yaml')))
        self.cfg = read_config(path)
        self.rate = float(self.cfg['decode_hz'])
        self.ground_test = (rospy.get_param('~ground_test', False) is True)
        self.stability = ColorStability(
            stable_frames=int(self.cfg['stable_frames']),
            freshness_s=float(self.cfg['freshness_s']),
            max_jump_px=float(self.cfg['max_centroid_jump_px']))
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.last_process = 0.0
        # Ground-test/ARM gates are unchanged. This independent window
        # gates actual color decoding, not merely the console display.
        self.window_active = False
        rospy.Subscriber(NS + '/demo7/color/window_active', Bool,
                         self.on_color_window, queue_size=1)
        self.px4_armed = False
        self.px4_offboard = False
        self.px4_last_seen = None
        self.camera_info = None
        self.depth_history = deque(maxlen=10)
        self.odom_history = deque(maxlen=100)
        self.extrinsic = None
        self.extrinsic_seen = None
        self.publisher = rospy.Publisher(TOPIC, String, queue_size=2, latch=False)
        self.world_publishers = {
            color: rospy.Publisher(NS + '/demo7/color/' + color + '/center_world',
                                   PointStamped, queue_size=1, latch=False)
            for color in COLORS
        }

        rospy.Subscriber(NS + '/camera/color/image_raw', Image,
                         self.on_image, queue_size=1, buff_size=4 * 1024 * 1024)
        rospy.Subscriber(NS + '/camera/aligned_depth_to_color/image_raw', Image,
                         self.on_depth, queue_size=2, buff_size=4 * 1024 * 1024)
        rospy.Subscriber(NS + '/camera/color/camera_info', CameraInfo,
                         self.on_camera_info, queue_size=1)
        rospy.Subscriber(NS + '/mavros/local_position/odom', Odometry,
                         self.on_odom, queue_size=10)
        rospy.Subscriber(NS + '/vins_fusion/extrinsic', Odometry,
                         self.on_extrinsic, queue_size=1)
        rospy.Subscriber(NS + '/mavros/state', State,
                         self.on_state, queue_size=1)
        rospy.loginfo('[DEMO7 COLOR] ready: <=%.1fHz, stable=%d, ground_test=%s',
                      self.rate, self.stability.stable_frames, self.ground_test)

    def on_color_window(self, msg):
        with self.lock:
            next_active = bool(msg.data)
            if next_active != self.window_active:
                self.window_active = next_active
                self.stability.process([], time.monotonic())
        if not next_active:
            self.publisher.publish(String(data='{"targets":[]}'))

    def on_state(self, msg):
        with self.lock:
            self.px4_armed = bool(msg.connected and msg.armed)
            self.px4_offboard = (msg.mode == 'OFFBOARD')
            self.px4_last_seen = time.monotonic()

    def on_depth(self, msg):
        if stamp_s(msg) is not None:
            with self.lock:
                self.depth_history.append(msg)

    def on_camera_info(self, msg):
        if (msg.width > 0 and msg.height > 0 and
                math.isfinite(msg.K[0]) and msg.K[0] > 0 and
                math.isfinite(msg.K[4]) and msg.K[4] > 0):
            with self.lock:
                self.camera_info = msg

    def on_odom(self, msg):
        if stamp_s(msg) is not None:
            with self.lock:
                self.odom_history.append(msg)

    def on_extrinsic(self, msg):
        if stamp_s(msg) is not None:
            with self.lock:
                self.extrinsic = msg
                self.extrinsic_seen = time.monotonic()

    def _active(self, now):
        with self.lock:
            fresh = (self.px4_last_seen is not None and
                     now - self.px4_last_seen <= 2.0)
            return self.ground_test or (fresh and self.px4_armed and self.px4_offboard)

    def _find_blobs(self, bgr):
        """Return per-color candidates and their contours in the RGB image."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        height, width = hsv.shape[:2]
        c = self.cfg
        smin = int(c['saturation_min'])
        vmin = int(c['value_min'])
        open_kernel = np.ones((int(c['morph_open_size']),) * 2, dtype=np.uint8)
        close_kernel = np.ones((int(c['morph_close_size']),) * 2, dtype=np.uint8)
        found = {}
        for color in COLORS:
            mask = np.zeros((height, width), dtype=np.uint8)
            for lo, hi in c['hue_ranges'][color]:
                mask = cv2.bitwise_or(
                    mask, cv2.inRange(hsv, (lo, smin, vmin), (hi, 255, 255)))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[-2]
            candidates = []
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if (area < float(c['min_contour_area_px']) or
                        area > float(c['max_contour_area_fraction']) * width * height):
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                if w < int(c['min_blob_width_px']) or h < int(c['min_blob_height_px']):
                    continue
                moment = cv2.moments(contour)
                if moment['m00'] <= 1e-9:
                    continue
                u = float(moment['m10'] / moment['m00'])
                v = float(moment['m01'] / moment['m00'])
                data = {'color': color, 'pixel_xy': [round(u, 2), round(v, 2)],
                        'area_px': round(area, 1), 'bbox_xywh': [x, y, w, h]}
                candidates.append((data, contour))
            found[color] = candidates
        return found

    def _geometry(self, rgb_msg, color_contour):
        """Return RGB-center-aligned depth and PX4 world position or None.

        Same time/freshness and quaternion convention as demo7_qr_monitor.py.
        Median is taken ONLY inside an eroded color mask, not on background.
        """
        t = stamp_s(rgb_msg)
        if t is None:
            return None
        with self.lock:
            info = self.camera_info
            depths = tuple(self.depth_history)
            odoms = tuple(self.odom_history)
            ext = self.extrinsic
            ext_seen = self.extrinsic_seen
        if (info is None or not depths or not odoms or ext is None or
                ext_seen is None or
                time.monotonic() - ext_seen > float(self.cfg['max_extrinsic_age_s'])):
            return None
        if info.width != rgb_msg.width or info.height != rgb_msg.height:
            return None
        depth_msg = min(depths, key=lambda m: abs(stamp_s(m) - t))
        odom = min(odoms, key=lambda m: abs(stamp_s(m) - t))
        if (abs(stamp_s(depth_msg) - t) > float(self.cfg['max_sensor_skew_s']) or
                abs(stamp_s(odom) - t) > float(self.cfg['max_sensor_skew_s']) or
                depth_msg.width != rgb_msg.width or
                depth_msg.height != rgb_msg.height or
                depth_msg.encoding not in ('16UC1', '32FC1') or
                not odom.header.frame_id):
            return None

        # 与 QR 一样：取目标中心附近 9x9 的深度中位数
        # 但额外要求像素处于颜色轮廓内部，避免背景干扰
        depth = self.bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding='passthrough'
        )

        observation = color_contour[0]
        contour = color_contour[1]

        u, v = observation['pixel_xy']
        if not (math.isfinite(u) and math.isfinite(v)):
            return None

        ix, iy = int(round(u)), int(round(v))
        rad = 4  # 9x9，与二维码一致

        x0 = max(0, ix - rad)
        x1 = min(depth_msg.width, ix + rad + 1)
        y0 = max(0, iy - rad)
        y1 = min(depth_msg.height, iy + rad + 1)

        if x0 >= x1 or y0 >= y1:
            return None

        # 生成颜色轮廓掩膜
        mask = np.zeros(
            (depth_msg.height, depth_msg.width),
            dtype=np.uint8
        )
        cv2.drawContours(mask, [contour], -1, 255, thickness=-1)

        erode_px = int(self.cfg['depth_interior_erode_px'])
        if erode_px:
            kernel = np.ones(
                (2 * erode_px + 1, 2 * erode_px + 1),
                dtype=np.uint8
            )
            mask = cv2.erode(mask, kernel)

        # 只取质心附近 9x9 的有效颜色区域深度
        patch = np.asarray(
            depth[y0:y1, x0:x1],
            dtype=np.float64
        )

        if depth_msg.encoding == '16UC1':
            patch *= 0.001

        valid = patch[
            (mask[y0:y1, x0:x1] > 0) &
            np.isfinite(patch) &
            (patch >= float(self.cfg['min_depth_m'])) &
            (patch <= float(self.cfg['max_depth_m']))
        ]

        # 和 QR 一样，至少需要 5 个有效深度像素
        if valid.size < 5:
            return None

        z = float(np.median(valid))
        fx, fy, cx, cy = map(float, (info.K[0], info.K[4], info.K[2], info.K[5]))
        p_camera = np.asarray([(u-cx)*z/fx, (v-cy)*z/fy, z], dtype=np.float64)
        ep = ext.pose.pose.position
        bp = odom.pose.pose.position
        p_body = np.asarray([ep.x, ep.y, ep.z]) + rotate(
            ext.pose.pose.orientation, p_camera)
        p_world = np.asarray([bp.x, bp.y, bp.z]) + rotate(
            odom.pose.pose.orientation, p_body)
        if not np.all(np.isfinite(p_world)):
            return None

        p = PointStamped()
        p.header.stamp = rgb_msg.header.stamp
        p.header.frame_id = odom.header.frame_id
        p.point.x, p.point.y, p.point.z = map(float, p_world)
        return p, {
            'depth_z_m': round(z, 4),
            'camera_xyz_m': [round(float(n), 4) for n in p_camera],
            'world_xyz_m': [round(float(n), 4) for n in p_world],
            'frame_id': p.header.frame_id,
            'rgb_depth_skew_s': round(abs(stamp_s(depth_msg) - t), 4),
            'rgb_odom_skew_s': round(abs(stamp_s(odom) - t), 4),
        }

    def on_image(self, msg):
        now = time.monotonic()
        with self.lock:
            if not self.window_active or now - self.last_process < 1.0 / self.rate:
                return
            self.last_process = now
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            blobs = self._find_blobs(rgb)
            obs = [pair[0] for color in COLORS for pair in blobs[color]]
            with self.lock:
                if not self.window_active:
                    return
                stable = self.stability.process(obs, now)
            if not stable or not self._active(time.monotonic()):
                # Explicitly invalidate the fourth terminal's previous color
                # results. Empty ROS messages never appear on the console.
                self.publisher.publish(String(data='{"targets":[]}'))
                return
            targets = []
            for color in COLORS:
                if color not in stable or len(blobs[color]) != 1:
                    continue
                data = dict(stable[color])
                data.update({'world_xyz_m': None, 'frame_id': None,
                             'depth_z_m': None, 'camera_xyz_m': None})
                try:
                    geometry = self._geometry(msg, blobs[color][0])
                    if geometry is not None:
                        point, extra = geometry
                        data.update(extra)
                        self.world_publishers[color].publish(point)
                except Exception as exc:
                    rospy.logwarn_throttle(5.0, '[DEMO7 COLOR] geometry: %s', str(exc))
                targets.append(data)
            if targets:
                with self.lock:
                    if not self.window_active:
                        return
                self.publisher.publish(String(data=json.dumps({
                    'stamp_ros_s': stamp_s(msg), 'targets': targets,
                }, ensure_ascii=False)))
        except Exception as exc:
            self.stability.process([], time.monotonic())
            rospy.logerr_throttle(5.0, '[DEMO7 COLOR] RGB processing error: %s', str(exc))


def main():
    rospy.init_node('demo7_color_monitor', anonymous=False)
    ColorMonitor()
    rospy.spin()


if __name__ == '__main__':
    main()
