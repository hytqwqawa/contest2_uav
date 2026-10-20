#!/usr/bin/env python3

import math
import rospy

from nav_msgs.msg import Odometry


class T265NaNGuard:

    def __init__(self):

        self.raw_topic = rospy.get_param(
            "~raw_topic",
            "/contest2/t265/odom/sample"
        )

        self.filtered_topic = rospy.get_param(
            "~filtered_topic",
            "/contest2/t265/odom/filtered"
        )

        self.pass_count = 0
        self.drop_count = 0

        self.pub = rospy.Publisher(
            self.filtered_topic,
            Odometry,
            queue_size=20
        )

        self.sub = rospy.Subscriber(
            self.raw_topic,
            Odometry,
            self.callback,
            queue_size=20
        )

        rospy.logwarn(
            "T265_NAN_GUARD_V1 active: %s -> %s",
            self.raw_topic,
            self.filtered_topic
        )

    def callback(self, msg):

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        values = [
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w
        ]

        # Never propagate NaN / Inf downstream.
        if not all(math.isfinite(v) for v in values):

            self.drop_count += 1

            rospy.logerr_throttle(
                0.5,
                "T265_NAN_GUARD_V1 DROP: "
                "non-finite pose detected "
                "(dropped=%d)" % self.drop_count
            )
            return

        qnorm2 = (
            q.x * q.x +
            q.y * q.y +
            q.z * q.z +
            q.w * q.w
        )

        # Invalid/corrupted quaternion. Normal T265 quaternion
        # is unit length. Do not modify it; reject bad samples.
        if (
            not math.isfinite(qnorm2)
            or qnorm2 < 0.64
            or qnorm2 > 1.44
        ):

            self.drop_count += 1

            rospy.logerr_throttle(
                0.5,
                "T265_NAN_GUARD_V1 DROP: "
                "invalid quaternion norm=%.6f "
                "(dropped=%d)"
                % (math.sqrt(max(qnorm2, 0.0)),
                   self.drop_count)
            )
            return

        self.pass_count += 1

        # Valid data are forwarded unchanged.
        self.pub.publish(msg)


if __name__ == "__main__":

    rospy.init_node("t265_nan_guard")

    T265NaNGuard()

    rospy.spin()
