#!/usr/bin/env python3
"""Demo7 independent flight-wide QR observer. NO flight commands/services.

RGB from D435i is decoded at <=5Hz; stable, fresh QR values are published and
printed every 1s in ANSI yellow during PX4 ARMED+OFFBOARD. No flight actuation.
"""
import json
import math
import sys
import threading
import time

import rospy
from cv_bridge import CvBridge
from mavros_msgs.msg import State
from pyzbar.pyzbar import ZBarSymbol, decode
from sensor_msgs.msg import Image
from std_msgs.msg import String
from qr_stability import QrStability

QR_TOPIC = '/contest2/demo7/qr/result'
STATUS_TOPIC = '/contest2/demo7/qr/status'
YELLOW = '\033[1;33m'
RESET = '\033[0m'


class QRMonitor:
    def __init__(self):
        self.rate = float(rospy.get_param('~decode_hz', 5.0))
        self.required = int(rospy.get_param('~stable_frames', 3))
        self.ground_test = bool(rospy.get_param('~ground_test', False))
        if not (0.5 <= self.rate <= 5.0 and 3 <= self.required <= 10):
            raise ValueError('QR rate or stability outside permitted range')
        self.bridge = CvBridge()
        self.qr = QrStability(min_frames=self.required, freshness_s=1.5)
        self.lock = threading.Lock()
        self.last_process = 0.0
        self.px4_armed = False
        self.px4_offboard = False
        self.px4_last_seen = None
        self.result_pub = rospy.Publisher(QR_TOPIC, String, queue_size=1, latch=False)
        self.status_pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=1, latch=False)
        self.image_sub = rospy.Subscriber(
            '/contest2/camera/color/image_raw', Image, self.on_image,
            queue_size=1, buff_size=4 * 1024 * 1024)
        self.px4_sub = rospy.Subscriber(
            '/contest2/mavros/state', State, self.on_state, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.on_second)
        rospy.loginfo('[DEMO7 QR] observer started; RGB %.1fHz decode, %d-frame stable, 1Hz yellow output; ground_test=%s',
                      self.rate, self.required, self.ground_test)

    def on_state(self, msg):
        with self.lock:
            self.px4_armed = bool(msg.connected and msg.armed)
            self.px4_offboard = (msg.mode == 'OFFBOARD')
            self.px4_last_seen = time.monotonic()

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
            for obj in objects:
                try:
                    text = obj.data.decode('utf-8', errors='strict')
                except UnicodeError:
                    continue
                if text and len(text) <= 512:
                    values.append(text)
            with self.lock:
                self.qr.process(values, time.monotonic())
        except Exception as exc:
            rospy.logerr_throttle(5.0, '[DEMO7 QR] frame processing error: %s', str(exc))
            with self.lock:
                self.qr.process([], time.monotonic())

    def on_second(self, _event):
        now = time.monotonic()
        with self.lock:
            fresh_px4 = (self.px4_last_seen is not None and now - self.px4_last_seen <= 2.0)
            in_flight = fresh_px4 and self.px4_armed and self.px4_offboard
            visible = self.qr.current(now)
            state = self.qr.status(now)
        active = in_flight or self.ground_test
        if not active:
            self.result_pub.publish(String(data=''))
            self.status_pub.publish(String(data='IDLE'))
            return
        self.result_pub.publish(String(data=visible))
        self.status_pub.publish(String(data=state))
        if visible:
            # json.dumps also escapes control characters coming from untrusted QR text.
            safe = json.dumps(visible, ensure_ascii=False)
            print(YELLOW + '[DEMO7 QR] 1Hz detected: ' + safe + RESET, flush=True)


if __name__ == '__main__':
    rospy.init_node('demo7_qr_monitor', anonymous=False)
    QRMonitor()
    rospy.spin()
