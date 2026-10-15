#!/usr/bin/env python3

import math
import rospy

from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, PositionTarget
from tf.transformations import euler_from_quaternion


class TrajectoryTestSource:

    def __init__(self):

        self.odom_topic = "/uav1/mavros/local_position/odom"
        self.state_topic = "/uav1/mavros/state"
        self.output_topic = "/ego_uav_interface/trajectory_setpoint"

        rospy.loginfo("Waiting for PX4 state...")
        state = rospy.wait_for_message(
            self.state_topic,
            State,
            timeout=5.0
        )

        if state.armed:
            rospy.logerr("Vehicle is ARMED. Test source will NOT start.")
            rospy.signal_shutdown("Vehicle armed")
            return

        if state.mode == "OFFBOARD":
            rospy.logerr("Vehicle is already in OFFBOARD. Test source will NOT start.")
            rospy.signal_shutdown("Vehicle in OFFBOARD")
            return

        rospy.loginfo("Waiting for odometry...")
        odom = rospy.wait_for_message(
            self.odom_topic,
            Odometry,
            timeout=5.0
        )

        self.state = state

        self.x0 = odom.pose.pose.position.x
        self.y0 = odom.pose.pose.position.y
        self.z0 = odom.pose.pose.position.z

        q = odom.pose.pose.orientation

        _, _, self.yaw0 = euler_from_quaternion([
            q.x,
            q.y,
            q.z,
            q.w
        ])

        self.pub = rospy.Publisher(
            self.output_topic,
            PositionTarget,
            queue_size=10
        )

        self.state_sub = rospy.Subscriber(
            self.state_topic,
            State,
            self.state_callback,
            queue_size=10
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / 30.0),
            self.timer_callback
        )

        rospy.logwarn("======================================")
        rospy.logwarn(" TRAJECTORY TEST SOURCE")
        rospy.logwarn("--------------------------------------")
        rospy.logwarn(
            "Captured p = [%.3f %.3f %.3f]",
            self.x0, self.y0, self.z0
        )
        rospy.logwarn(
            "Captured yaw = %.2f deg",
            math.degrees(self.yaw0)
        )
        rospy.logwarn("")
        rospy.logwarn("Test feedforward values:")
        rospy.logwarn("v = [ 0.12 -0.08  0.04 ] m/s")
        rospy.logwarn("a = [ 0.20 -0.10  0.05 ] m/s^2")
        rospy.logwarn("yaw_rate = 0.15 rad/s")
        rospy.logwarn("")
        rospy.logwarn("ONLY FOR DISARMED POSCTL INTERFACE TEST")
        rospy.logwarn("======================================")

    def state_callback(self, msg):

        self.state = msg

        if msg.armed:
            rospy.logerr("ARM detected -> stopping test source")
            rospy.signal_shutdown("ARM detected")

        elif msg.mode == "OFFBOARD":
            rospy.logerr("OFFBOARD detected -> stopping test source")
            rospy.signal_shutdown("OFFBOARD detected")

    def timer_callback(self, event):

        if rospy.is_shutdown():
            return

        if self.state.armed or self.state.mode == "OFFBOARD":
            return

        cmd = PositionTarget()

        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "map"

        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # trajectory_interface will enforce its final mask.
        cmd.type_mask = 0

        # Position: current captured position
        cmd.position.x = self.x0
        cmd.position.y = self.y0
        cmd.position.z = self.z0

        # Distinct test velocity
        cmd.velocity.x = 0.12
        cmd.velocity.y = -0.08
        cmd.velocity.z = 0.04

        # Distinct test acceleration
        cmd.acceleration_or_force.x = 0.20
        cmd.acceleration_or_force.y = -0.10
        cmd.acceleration_or_force.z = 0.05

        cmd.yaw = self.yaw0
        cmd.yaw_rate = 0.15

        self.pub.publish(cmd)


if __name__ == "__main__":

    rospy.init_node("trajectory_test_source")

    try:
        TrajectoryTestSource()
        rospy.spin()

    except rospy.ROSException as exc:
        rospy.logerr(str(exc))
