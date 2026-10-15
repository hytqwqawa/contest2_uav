#!/usr/bin/env python3
import rospy

from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


class TrajectoryMux:

    def __init__(self):

        self.timeout = 0.25

        self.source = "POINT"

        self.point_msg = None
        self.point_time = None

        self.ego_msg = None
        self.ego_time = None

        self.pub = rospy.Publisher(
            "/contest2/ego_uav_interface/trajectory_setpoint",
            PositionTarget,
            queue_size=10
        )

        self.status_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/trajectory_mux_status",
            String,
            queue_size=1,
            latch=True
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/point_setpoint",
            PositionTarget,
            self.point_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/ego_setpoint",
            PositionTarget,
            self.ego_cb,
            queue_size=20
        )

        rospy.Service(
            "/contest2/ego_uav_interface/use_point",
            Trigger,
            self.use_point
        )

        rospy.Service(
            "/contest2/ego_uav_interface/use_ego",
            Trigger,
            self.use_ego
        )

        rospy.Timer(
            rospy.Duration(0.02),
            self.timer_cb
        )

    def point_cb(self, msg):
        self.point_msg = msg
        self.point_time = rospy.Time.now()

    def ego_cb(self, msg):
        self.ego_msg = msg
        self.ego_time = rospy.Time.now()

    def fresh(self, t):
        return (
            t is not None and
            (rospy.Time.now() - t).to_sec() < self.timeout
        )

    def use_point(self, req):

        if not self.fresh(self.point_time):
            return TriggerResponse(
                False,
                "POINT source unavailable/stale"
            )

        self.source = "POINT"

        return TriggerResponse(
            True,
            "Trajectory source -> POINT"
        )

    def use_ego(self, req):

        if not self.fresh(self.ego_time):
            return TriggerResponse(
                False,
                "EGO source unavailable/stale"
            )

        self.source = "EGO"

        return TriggerResponse(
            True,
            "Trajectory source -> EGO"
        )

    def timer_cb(self, event):

        if self.source == "POINT":

            if self.fresh(self.point_time):
                self.pub.publish(self.point_msg)
                status = "POINT_ACTIVE"
            else:
                status = "POINT_STALE"

        else:

            if self.fresh(self.ego_time):
                self.pub.publish(self.ego_msg)
                status = "EGO_ACTIVE"
            else:
                status = "EGO_STALE"

        self.status_pub.publish(
            String(data=status)
        )


if __name__ == "__main__":
    rospy.init_node("trajectory_mux")
    TrajectoryMux()
    rospy.spin()
