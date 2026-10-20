#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry


class CameraExtrinsicPublisher:

    def __init__(self):

        self.topic = rospy.get_param(
            "~topic",
            "/contest2/vins_fusion/extrinsic"
        )

        # Camera optical center expressed in body FLU frame:
        # +x forward
        # +y left
        # +z up
        self.tx = rospy.get_param("~x", 0.01)
        self.ty = rospy.get_param("~y", 0.00)
        self.tz = rospy.get_param("~z", 0.04)

        self.rate_hz = rospy.get_param(
            "~rate",
            10.0
        )

        self.pub = rospy.Publisher(
            self.topic,
            Odometry,
            queue_size=1,
            latch=True
        )

        rospy.loginfo("======================================")
        rospy.loginfo(" Camera extrinsic publisher")
        rospy.loginfo("--------------------------------------")
        rospy.loginfo(" Topic: %s", self.topic)
        rospy.loginfo("")
        rospy.loginfo(" T_BC translation [body frame]:")
        rospy.loginfo(
            "   x = %.3f m (forward)",
            self.tx
        )
        rospy.loginfo(
            "   y = %.3f m (left)",
            self.ty
        )
        rospy.loginfo(
            "   z = %.3f m (up)",
            self.tz
        )
        rospy.loginfo("")
        rospy.loginfo(" Optical -> body rotation:")
        rospy.loginfo("   body_x =  camera_z")
        rospy.loginfo("   body_y = -camera_x")
        rospy.loginfo("   body_z = -camera_y")
        rospy.loginfo("======================================")

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate_hz),
            self.timer_callback
        )

    def timer_callback(self, event):

        msg = Odometry()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.child_frame_id = "camera_depth_optical_frame"

        # Translation of camera origin in body coordinates
        msg.pose.pose.position.x = self.tx
        msg.pose.pose.position.y = self.ty
        msg.pose.pose.position.z = self.tz

        # Rotation R_BC:
        #
        # [ 0  0  1 ]
        # [-1  0  0 ]
        # [ 0 -1  0 ]
        #
        # Quaternion (x, y, z, w)
        msg.pose.pose.orientation.x = -0.5
        msg.pose.pose.orientation.y =  0.5
        msg.pose.pose.orientation.z = -0.5
        msg.pose.pose.orientation.w =  0.5

        self.pub.publish(msg)


if __name__ == "__main__":

    rospy.init_node(
        "camera_extrinsic_publisher"
    )

    CameraExtrinsicPublisher()

    rospy.spin()
