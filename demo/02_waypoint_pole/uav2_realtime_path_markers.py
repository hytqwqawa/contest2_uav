#!/usr/bin/env python3

import math
import threading
from pathlib import Path

import rospy
import yaml

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String, Empty
from visualization_msgs.msg import Marker, MarkerArray


class RealtimePathMarkers:

    def __init__(self):

        self.lock = threading.RLock()

        self.active = False
        self.leg_index = -1

        self.desired = []
        self.actual = []

        self.last_desired = None
        self.last_actual = None

        root = Path(__file__).resolve().parent

        with open(
            root / "session_frame.yaml",
            "r"
        ) as f:
            session = yaml.safe_load(f)

        o = session["origin_world"]

        self.ox = float(o["x"])
        self.oy = float(o["y"])
        self.oz = float(o["z"])

        self.flight_h = float(
            session.get(
                "flight_height",
                0.8
            )
        )

        self.pub = rospy.Publisher(
            "/contest2/yaw_tangent_demo/realtime_paths",
            MarkerArray,
            queue_size=1,
            latch=True
        )

        rospy.Subscriber(
            "/contest2/position_cmd",
            PositionCommand,
            self.cmd_cb,
            queue_size=100
        )

        rospy.Subscriber(
            "/contest2/mavros/local_position/odom",
            Odometry,
            self.odom_cb,
            queue_size=100
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/rviz_goal_status",
            String,
            self.status_cb,
            queue_size=100
        )


        rospy.Subscriber(
            "/contest2/waypoint_pole/reset_live_paths",
            Empty,
            self.reset_cb,
            queue_size=1
        )

        self.timer = rospy.Timer(
            rospy.Duration(0.10),
            self.timer_cb
        )

        rospy.logwarn(
            "[LIVE PATH] ready: "
            "DESIRED=/contest2/position_cmd, "
            "ACTUAL=/contest2/mavros/local_position/odom"
        )


    @staticmethod
    def point(x, y, z):

        p = Point()

        p.x = x
        p.y = y
        p.z = z

        return p


    @staticmethod
    def moved(last, p, threshold=0.006):

        if last is None:
            return True

        return math.sqrt(
            (p[0] - last[0]) ** 2
            +
            (p[1] - last[1]) ** 2
            +
            (p[2] - last[2]) ** 2
        ) >= threshold


    def reset_cb(self, msg):

        with self.lock:

            self.active = False
            self.leg_index = -1

            self.desired = []
            self.actual = []

            self.last_desired = None
            self.last_actual = None

        # Explicitly clear the previous mission's RViz paths.
        arr = MarkerArray()

        m = Marker()
        m.action = Marker.DELETEALL

        arr.markers.append(m)

        self.pub.publish(arr)

        rospy.logwarn(
            "[LIVE PATH] NEW MISSION RESET"
        )


    def status_cb(self, msg):

        text = msg.data

        with self.lock:

            if text.startswith(
                "TRANSLATE traj_id="
            ):

                self.leg_index += 1
                self.active = True

                self.desired.append([])
                self.actual.append([])

                self.last_desired = None
                self.last_actual = None

                rospy.logwarn(
                    "[LIVE PATH] LEG %d recording started",
                    self.leg_index + 1
                )

            elif (
                text.startswith("DONE ")
                or
                text.startswith("ABORT")
            ):

                self.active = False

                rospy.logwarn(
                    "[LIVE PATH] LEG %d recording stopped",
                    self.leg_index + 1
                )


    def cmd_cb(self, msg):

        with self.lock:

            if (
                not self.active
                or
                self.leg_index < 0
            ):
                return

            p = (
                msg.position.x,
                msg.position.y,
                msg.position.z
            )

            if self.moved(
                self.last_desired,
                p
            ):

                self.desired[
                    self.leg_index
                ].append(p)

                self.last_desired = p


    def odom_cb(self, msg):

        with self.lock:

            if (
                not self.active
                or
                self.leg_index < 0
            ):
                return

            q = msg.pose.pose.position

            p = (
                q.x,
                q.y,
                q.z
            )

            if self.moved(
                self.last_actual,
                p
            ):

                self.actual[
                    self.leg_index
                ].append(p)

                self.last_actual = p


    def add_path(
        self,
        arr,
        ns,
        mid,
        pts,
        desired
    ):

        if len(pts) < 2:
            return

        m = Marker()

        m.header.frame_id = "world"
        m.header.stamp = rospy.Time.now()

        m.ns = ns
        m.id = mid

        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD

        if desired:

            # Orange
            m.scale.x = 0.035

            m.color.r = 1.0
            m.color.g = 0.55
            m.color.b = 0.0
            m.color.a = 1.0

        else:

            # Magenta
            m.scale.x = 0.045

            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.75
            m.color.a = 1.0

        for x, y, z in pts:

            m.points.append(
                self.point(
                    x,
                    y,
                    z + (
                        0.010
                        if desired
                        else 0.025
                    )
                )
            )

        arr.markers.append(m)


    def add_legend(
        self,
        arr,
        mid,
        text,
        z,
        desired
    ):

        m = Marker()

        m.header.frame_id = "world"
        m.header.stamp = rospy.Time.now()

        m.ns = "live_path_legend"
        m.id = mid

        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD

        m.pose.position.x = self.ox
        m.pose.position.y = self.oy
        m.pose.position.z = z

        m.pose.orientation.w = 1.0

        m.scale.z = 0.13

        if desired:

            m.color.r = 1.0
            m.color.g = 0.55
            m.color.b = 0.0

        else:

            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.75

        m.color.a = 1.0
        m.text = text

        arr.markers.append(m)


    def timer_cb(self, event):

        arr = MarkerArray()

        with self.lock:

            for i, pts in enumerate(
                self.desired
            ):

                self.add_path(
                    arr,
                    "live_desired",
                    3000 + i,
                    pts,
                    True
                )

            for i, pts in enumerate(
                self.actual
            ):

                self.add_path(
                    arr,
                    "live_actual",
                    3100 + i,
                    pts,
                    False
                )

        self.add_legend(
            arr,
            3200,
            "DESIRED LIVE  /contest2/position_cmd",
            self.oz + self.flight_h + 0.38,
            True
        )

        self.add_legend(
            arr,
            3201,
            "ACTUAL LIVE  /odom",
            self.oz + self.flight_h + 0.22,
            False
        )

        self.pub.publish(arr)


if __name__ == "__main__":

    rospy.init_node(
        "uav2_realtime_path_markers"
    )

    RealtimePathMarkers()

    rospy.spin()
