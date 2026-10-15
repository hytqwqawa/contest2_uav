#!/usr/bin/env python3

import math
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from mavros_msgs.msg import PositionTarget
from tf.transformations import euler_from_quaternion


class OffboardHoldNode:

    def __init__(self):

        self.odom_topic = rospy.get_param(
            "~odom_topic",
            "/uav1/mavros/local_position/odom"
        )

        self.health_topic = rospy.get_param(
            "~health_topic",
            "/ego_uav_interface/flight_healthy"
        )

        self.setpoint_topic = rospy.get_param(
            "~setpoint_topic",
            "/uav1/mavros/setpoint_raw/local"
        )

        self.odom_timeout = rospy.get_param(
            "~odom_timeout",
            0.20
        )

        self.health_timeout = rospy.get_param(
            "~health_timeout",
            0.30
        )

        self.publish_rate = rospy.get_param(
            "~publish_rate",
            30.0
        )

        self.stream_ready_time = rospy.get_param(
            "~stream_ready_time",
            1.5
        )

        self.odom = None
        self.odom_time = None

        self.flight_healthy = False
        self.health_time = None

        self.hold_captured = False
        self.stream_ready = False

        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z = 0.0
        self.hold_yaw = 0.0

        self.stream_start_time = None

        self.odom_sub = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=20
        )

        self.health_sub = rospy.Subscriber(
            self.health_topic,
            Bool,
            self.health_callback,
            queue_size=20
        )

        self.setpoint_pub = rospy.Publisher(
            self.setpoint_topic,
            PositionTarget,
            queue_size=10
        )

        self.ready_pub = rospy.Publisher(
            "/ego_uav_interface/offboard_stream_ready",
            Bool,
            queue_size=1,
            latch=True
        )

        self.status_pub = rospy.Publisher(
            "/ego_uav_interface/offboard_stream_status",
            String,
            queue_size=1,
            latch=True
        )

        rospy.loginfo("======================================")
        rospy.loginfo(" ego_uav_interface offboard hold PY")
        rospy.loginfo("--------------------------------------")
        rospy.loginfo(" Odom topic     : %s", self.odom_topic)
        rospy.loginfo(" Health topic   : %s", self.health_topic)
        rospy.loginfo(" Setpoint topic : %s", self.setpoint_topic)
        rospy.loginfo(" Publish rate   : %.1f Hz", self.publish_rate)
        rospy.loginfo("--------------------------------------")
        rospy.loginfo(" DOES NOT ARM")
        rospy.loginfo(" DOES NOT SWITCH MODE")
        rospy.loginfo("======================================")

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self.timer_callback
        )

    def odom_callback(self, msg):
        self.odom = msg
        self.odom_time = rospy.Time.now()

    def health_callback(self, msg):
        self.flight_healthy = msg.data
        self.health_time = rospy.Time.now()

    def capture_hold_point(self):

        self.hold_x = self.odom.pose.pose.position.x
        self.hold_y = self.odom.pose.pose.position.y
        self.hold_z = self.odom.pose.pose.position.z

        q = self.odom.pose.pose.orientation

        _, _, self.hold_yaw = euler_from_quaternion([
            q.x,
            q.y,
            q.z,
            q.w
        ])

        self.stream_start_time = rospy.Time.now()

        self.hold_captured = True
        self.stream_ready = False

        rospy.logwarn(
            "HOLD POINT CAPTURED: "
            "x=%.3f y=%.3f z=%.3f yaw=%.2f deg",
            self.hold_x,
            self.hold_y,
            self.hold_z,
            math.degrees(self.hold_yaw)
        )

    def publish_setpoint(self):

        cmd = PositionTarget()

        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "map"

        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        cmd.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        cmd.position.x = self.hold_x
        cmd.position.y = self.hold_y
        cmd.position.z = self.hold_z

        cmd.yaw = self.hold_yaw

        self.setpoint_pub.publish(cmd)

    def publish_status(self, text):

        self.ready_pub.publish(
            Bool(data=self.stream_ready)
        )

        self.status_pub.publish(
            String(data=text)
        )

    def reset_stream(self):
        self.hold_captured = False
        self.stream_ready = False
        self.stream_start_time = None

    def timer_callback(self, event):

        now = rospy.Time.now()

        odom_fresh = (
            self.odom_time is not None and
            (now - self.odom_time).to_sec()
            < self.odom_timeout
        )

        health_fresh = (
            self.health_time is not None and
            (now - self.health_time).to_sec()
            < self.health_timeout
        )

        health_ok = (
            health_fresh and
            self.flight_healthy
        )

        if not odom_fresh:

            self.reset_stream()
            self.publish_status("WAIT_ODOM")

            rospy.logwarn_throttle(
                1.0,
                "[OFFBOARD STREAM] WAIT_ODOM"
            )

            return

        if not health_ok:

            self.reset_stream()
            self.publish_status(
                "WAIT_FLIGHT_HEALTHY"
            )

            rospy.logwarn_throttle(
                1.0,
                "[OFFBOARD STREAM] WAIT_FLIGHT_HEALTHY"
            )

            return

        if not self.hold_captured:
            self.capture_hold_point()

        self.publish_setpoint()

        streaming_time = (
            now - self.stream_start_time
        ).to_sec()

        if streaming_time >= self.stream_ready_time:
            self.stream_ready = True

        if self.stream_ready:
            status = "OFFBOARD_STREAM_READY"
        else:
            status = "STREAMING_SETPOINTS"

        self.publish_status(status)

        rospy.loginfo_throttle(
            1.0,
            "[OFFBOARD STREAM] "
            "hold=[%.3f %.3f %.3f] "
            "yaw=%.2f deg "
            "stream_time=%.2f s "
            "READY=%s",
            self.hold_x,
            self.hold_y,
            self.hold_z,
            math.degrees(self.hold_yaw),
            streaming_time,
            "YES" if self.stream_ready else "NO"
        )


if __name__ == "__main__":

    rospy.init_node("offboard_hold_node")

    node = OffboardHoldNode()

    rospy.spin()
