#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

INPUT_TOPIC  = "/contest2/mavros/vision_pose/pose"
OUTPUT_TOPIC = "/contest2/ego_uav_interface/external_pose_ref"

# PX4 EKF2_EV_POS, body FRD
EV_X =  0.05
EV_Y =  0.00
EV_Z = -0.09

# Convert FRD -> ROS FLU
LEVER_FLU = (EV_X, -EV_Y, -EV_Z)


def rotate_vec(q, v):
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = v

    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)

    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)

    return rx, ry, rz


class ExternalPoseAdapter:
    def __init__(self):
        self.pub = rospy.Publisher(
            OUTPUT_TOPIC,
            PoseStamped,
            queue_size=10
        )

        self.sub = rospy.Subscriber(
            INPUT_TOPIC,
            PoseStamped,
            self.cb,
            queue_size=20
        )

        rospy.loginfo("UAV2 external pose adapter started")
        rospy.loginfo("input : %s", INPUT_TOPIC)
        rospy.loginfo("output: %s", OUTPUT_TOPIC)
        rospy.loginfo(
            "EKF2_EV_POS FRD = [%.3f %.3f %.3f] m",
            EV_X, EV_Y, EV_Z
        )

    def cb(self, msg):
        out = PoseStamped()
        out.header = msg.header

        q = msg.pose.orientation
        dx, dy, dz = rotate_vec(q, LEVER_FLU)

        # Convert EV sensor position -> vehicle body origin
        out.pose.position.x = msg.pose.position.x - dx
        out.pose.position.y = msg.pose.position.y - dy
        out.pose.position.z = msg.pose.position.z - dz

        # Yaw/orientation already agrees with PX4 after EKF alignment
        out.pose.orientation = msg.pose.orientation

        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("uav2_external_pose_adapter")
    ExternalPoseAdapter()
    rospy.spin()
