#!/usr/bin/env python3

import csv
import math
import os
import threading

import rospy

from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String


def norm3(x, y, z):
    return math.sqrt(x*x + y*y + z*z)


def norm2(x, y):
    return math.sqrt(x*x + y*y)


class SpeedProbe:

    def __init__(self):

        self.csv_path = rospy.get_param(
            "~csv",
            "/tmp/uav2_speed_compare.csv"
        )

        self.summary_path = rospy.get_param(
            "~summary",
            "/tmp/uav2_speed_compare.summary.txt"
        )

        os.makedirs(
            os.path.dirname(self.csv_path),
            exist_ok=True
        )

        self.lock = threading.Lock()

        self.ego_active = False

        self.latest_cmd_3d = 0.0
        self.latest_cmd_xy = 0.0

        self.max_cmd_3d = 0.0
        self.max_cmd_xy = 0.0

        self.max_odom_3d = 0.0
        self.max_odom_xy = 0.0

        self.cmd_at_odom_peak = 0.0
        self.cmd_xy_at_odom_peak = 0.0

        self.closed = False

        self.f = open(
            self.csv_path,
            "w",
            newline=""
        )

        self.writer = csv.writer(self.f)

        self.writer.writerow([
            "time",
            "ego_active",
            "cmd_v3",
            "cmd_vxy",
            "odom_v3",
            "odom_vxy"
        ])

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
            queue_size=30
        )

        rospy.on_shutdown(
            self.shutdown
        )

        rospy.loginfo(
            "UAV2 speed probe ready"
        )


    def status_cb(self, msg):

        s = msg.data.strip()

        with self.lock:

            if (
                s.startswith("TRANSLATE")
                or
                s.startswith("RUNNING")
            ):
                self.ego_active = True

            elif (
                s.startswith("DONE")
                or
                s.startswith("ABORT")
                or
                s == "IDLE"
            ):
                self.ego_active = False


    def cmd_cb(self, msg):

        v = msg.velocity

        v3 = norm3(
            v.x,
            v.y,
            v.z
        )

        vxy = norm2(
            v.x,
            v.y
        )

        with self.lock:

            self.latest_cmd_3d = v3
            self.latest_cmd_xy = vxy

            if self.ego_active:

                self.max_cmd_3d = max(
                    self.max_cmd_3d,
                    v3
                )

                self.max_cmd_xy = max(
                    self.max_cmd_xy,
                    vxy
                )


    def odom_cb(self, msg):

        v = msg.twist.twist.linear

        odom3 = norm3(
            v.x,
            v.y,
            v.z
        )

        odomxy = norm2(
            v.x,
            v.y
        )

        now = rospy.Time.now().to_sec()

        with self.lock:

            active = self.ego_active

            cmd3 = self.latest_cmd_3d
            cmdxy = self.latest_cmd_xy

            self.writer.writerow([
                "%.6f" % now,
                int(active),
                "%.6f" % cmd3,
                "%.6f" % cmdxy,
                "%.6f" % odom3,
                "%.6f" % odomxy
            ])

            if active:

                if odom3 > self.max_odom_3d:

                    self.max_odom_3d = odom3

                    self.cmd_at_odom_peak = cmd3

                    self.cmd_xy_at_odom_peak = cmdxy

                self.max_odom_xy = max(
                    self.max_odom_xy,
                    odomxy
                )


    def shutdown(self):

        with self.lock:

            if self.closed:
                return

            self.closed = True

            self.f.flush()
            self.f.close()

            lines = [
                "========== SPEED PROBE SUMMARY ==========",
                "EGO-only data",
                "",
                "max_position_cmd_3d = %.3f m/s"
                % self.max_cmd_3d,
                "max_position_cmd_xy = %.3f m/s"
                % self.max_cmd_xy,
                "",
                "max_odom_3d         = %.3f m/s"
                % self.max_odom_3d,
                "max_odom_xy         = %.3f m/s"
                % self.max_odom_xy,
                "",
                "cmd_3d_at_odom_peak = %.3f m/s"
                % self.cmd_at_odom_peak,
                "cmd_xy_at_odom_peak = %.3f m/s"
                % self.cmd_xy_at_odom_peak,
                "",
                "CSV: %s"
                % self.csv_path,
                "========================================="
            ]

            text = "\n".join(lines)

            with open(
                self.summary_path,
                "w"
            ) as f:
                f.write(text + "\n")

            print(
                "\n" + text,
                flush=True
            )


if __name__ == "__main__":

    rospy.init_node(
        "uav2_speed_probe"
    )

    SpeedProbe()

    rospy.spin()
