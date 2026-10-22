#!/usr/bin/env python3

import copy
import math

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from mavros_msgs.msg import State


class LandingSanitizer:

    def __init__(self):

        self.input_topic = rospy.get_param(
            "~input_topic",
            "/contest2/t265/odom/filtered"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/contest2/t265/odom/landing_safe_dev"
        )

        self.range_topic = rospy.get_param(
            "~range_topic",
            "/contest2/mavros/distance_sensor/hrlv_ez4_pub"
        )

        self.state_topic = rospy.get_param(
            "~state_topic",
            "/contest2/mavros/state"
        )

        self.lock_range = rospy.get_param(
            "~lock_range",
            0.10
        )

        self.airborne_range = rospy.get_param(
            "~airborne_range",
            0.30
        )

        self.range_timeout = rospy.get_param(
            "~range_timeout",
            0.20
        )

        self.max_terminal_drop = rospy.get_param(
            "~max_terminal_drop",
            0.12
        )

        self.armed = False
        self.airborne_seen = False

        self.range_value = None
        self.range_time = None

        self.locked = False

        self.anchor_x = 0.0
        self.anchor_y = 0.0
        self.anchor_z = 0.0
        self.anchor_range = 0.0

        self.safe_z = 0.0

        self.pub = rospy.Publisher(
            self.output_topic,
            Odometry,
            queue_size=30
        )

        rospy.Subscriber(
            self.state_topic,
            State,
            self.state_cb,
            queue_size=20
        )

        rospy.Subscriber(
            self.range_topic,
            Range,
            self.range_cb,
            queue_size=30
        )

        rospy.Subscriber(
            self.input_topic,
            Odometry,
            self.odom_cb,
            queue_size=50
        )

        rospy.logwarn(
            "T265 LANDING SANITIZER DEV active: %s -> %s",
            self.input_topic,
            self.output_topic
        )


    def reset(self):

        self.airborne_seen = False
        self.locked = False

        self.anchor_x = 0.0
        self.anchor_y = 0.0
        self.anchor_z = 0.0
        self.anchor_range = 0.0

        self.safe_z = 0.0


    def state_cb(self, msg):

        was_armed = self.armed
        self.armed = bool(msg.armed)

        if was_armed and not self.armed:

            rospy.logwarn(
                "T265 LANDING SANITIZER DEV reset on DISARM"
            )

            self.reset()


    def range_cb(self, msg):

        r = float(msg.range)

        if not math.isfinite(r):
            return

        # Reject impossible/zero readings.
        if r < 0.03:
            return

        self.range_value = r
        self.range_time = rospy.Time.now()


    def range_fresh(self):

        return (
            self.range_value is not None
            and self.range_time is not None
            and (
                rospy.Time.now()
                - self.range_time
            ).to_sec() <= self.range_timeout
        )


    def odom_cb(self, msg):

        # -------------------------------
        # Normal mode: exact passthrough
        # -------------------------------

        if not self.armed:

            self.pub.publish(msg)
            return

        if self.range_fresh():

            if (
                not self.airborne_seen
                and self.range_value >= self.airborne_range
            ):

                self.airborne_seen = True

                rospy.loginfo(
                    "T265 LANDING SANITIZER DEV: airborne confirmed"
                )

        # -------------------------------
        # Final landing lock
        # -------------------------------

        if (
            not self.locked
            and self.airborne_seen
            and self.range_fresh()
            and self.range_value <= self.lock_range
        ):

            p = msg.pose.pose.position

            self.anchor_x = float(p.x)
            self.anchor_y = float(p.y)
            self.anchor_z = float(p.z)

            self.anchor_range = self.range_value
            self.safe_z = self.anchor_z

            self.locked = True

            rospy.logfatal(
                "T265 LANDING SANITIZER DEV LOCK: "
                "range=%.3f xyz=[%.3f %.3f %.3f]",
                self.anchor_range,
                self.anchor_x,
                self.anchor_y,
                self.anchor_z
            )

        if not self.locked:

            self.pub.publish(msg)
            return

        # -------------------------------------------------
        # From here on:
        #   X/Y are NEVER taken from drifting T265 again.
        #   Z follows only relative TFMini descent.
        # -------------------------------------------------

        out = copy.deepcopy(msg)

        out.pose.pose.position.x = self.anchor_x
        out.pose.pose.position.y = self.anchor_y

        if self.range_fresh():

            range_based_z = (
                self.anchor_z
                + (
                    self.range_value
                    - self.anchor_range
                )
            )

            z_floor = (
                self.anchor_z
                - self.max_terminal_drop
            )

            range_based_z = max(
                z_floor,
                range_based_z
            )

            # Monotonic descent only.
            # Never command estimator Z upward because
            # of ground-effect/range rebound.
            self.safe_z = min(
                self.safe_z,
                range_based_z
            )

        # If range becomes stale, hold the last safe Z.
        out.pose.pose.position.z = self.safe_z

        self.pub.publish(out)


if __name__ == "__main__":

    rospy.init_node(
        "t265_landing_sanitizer_dev"
    )

    LandingSanitizer()

    rospy.spin()
