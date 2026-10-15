#!/usr/bin/env python3

import math
import os
import subprocess
import time

import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


GREEN = "\033[32m"
RED = "\033[31m"
CYAN = "\033[36m"
RESET = "\033[0m"


class EgoSingleGoalDemo:

    def __init__(self):

        self.dx = rospy.get_param("~dx", 0.50)
        self.dy = rospy.get_param("~dy", 0.00)

        # Rectangle defined in the INITIAL BODY frame:
        # forward > 0, left > 0.
        self.half_forward = rospy.get_param(
            "~half_forward", 0.50
        )

        self.half_left = rospy.get_param(
            "~half_left", 0.50
        )

        self.waypoint_pause = rospy.get_param(
            "~waypoint_pause", 0.80
        )

        self.takeoff_height = rospy.get_param(
            "~takeoff_height", 0.80
        )

        self.altitude_tolerance = rospy.get_param(
            "~altitude_tolerance", 0.04
        )

        self.hover_time = rospy.get_param(
            "~hover_time", 2.0
        )

        self.odom = None
        self.state = None

        self.flight_healthy = False
        self.stream_ready = False
        self.mission_allowed = False

        self.point_status = ""
        self.mux_status = ""

        self.rviz_status = ""
        self.last_rviz_status = ""

        self.ego_started = False
        self.ego_done = False
        self.ego_abort = None

        rospy.Subscriber(
            "/uav1/mavros/local_position/odom",
            Odometry,
            self.odom_cb,
            queue_size=30
        )

        rospy.Subscriber(
            "/uav1/mavros/state",
            State,
            self.state_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/ego_uav_interface/flight_healthy",
            Bool,
            self.health_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/ego_uav_interface/offboard_stream_ready",
            Bool,
            self.stream_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/ego_uav_interface/rc_mission_allowed",
            Bool,
            self.mission_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/ego_uav_interface/point_traj_status",
            String,
            self.point_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/ego_uav_interface/trajectory_mux_status",
            String,
            self.mux_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/ego_uav_interface/rviz_goal_status",
            String,
            self.rviz_status_cb,
            queue_size=30
        )

        self.goal_pub = rospy.Publisher(
            "/ego_uav_interface/rviz_goal",
            PoseStamped,
            queue_size=1
        )

        self.yaw_mode_pub = rospy.Publisher(
            "/ego_uav_interface/yaw_mode",
            String,
            queue_size=1,
            latch=True
        )


    def odom_cb(self, msg):
        self.odom = msg


    def state_cb(self, msg):
        self.state = msg


    def health_cb(self, msg):
        self.flight_healthy = msg.data


    def stream_cb(self, msg):
        self.stream_ready = msg.data


    def mission_cb(self, msg):
        self.mission_allowed = msg.data


    def point_cb(self, msg):
        self.point_status = msg.data


    def mux_cb(self, msg):
        self.mux_status = msg.data


    def rviz_status_cb(self, msg):

        self.rviz_status = msg.data

        if self.rviz_status != self.last_rviz_status:
            print(
                "\n[EGO STATUS] %s"
                % self.rviz_status,
                flush=True
            )
            self.last_rviz_status = self.rviz_status

        if self.rviz_status.startswith("TRANSLATE"):
            self.ego_started = True

        if self.rviz_status.startswith("DONE"):
            self.ego_done = True

        if self.rviz_status.startswith("ABORT"):
            self.ego_abort = self.rviz_status


    def yaw(self):

        q = self.odom.pose.pose.orientation

        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )


    def safe_hold(self):

        for service in [
            "/ego_uav_interface/disable_trajectory",
            "/ego_uav_interface/abort_trajectory",
            "/ego_uav_interface/use_point",
        ]:

            try:
                rospy.wait_for_service(
                    service,
                    timeout=0.5
                )

                rospy.ServiceProxy(
                    service,
                    Trigger
                )()

            except Exception:
                pass


    def fail(self, reason):

        print()
        print(
            RED +
            "MISSION STOPPED: %s" % reason +
            RESET
        )

        print(
            "CH7 LOW / CH6 pilot takeover remains available."
        )

        self.safe_hold()

        raise RuntimeError(reason)


    def wait_stack(self):

        print()
        print("========================================")
        print(" UAV2 EGO SINGLE-GOAL DEMO")
        print("========================================")
        print(
            "Relative goal: dx=%+.2f m  dy=%+.2f m"
            % (self.dx, self.dy)
        )
        print("Yaw mode     : HOLD")
        print("Takeoff      : %.2f m" % self.takeoff_height)
        print()

        deadline = time.time() + 15.0

        while not rospy.is_shutdown():

            ready = (
                self.odom is not None
                and self.state is not None
                and self.state.connected
                and self.flight_healthy
                and self.stream_ready
                and self.point_status in [
                    "HOLD_GROUND",
                    "LANDED_HOLD"
                ]
                and self.goal_pub.get_num_connections() > 0
                and self.yaw_mode_pub.get_num_connections() > 0
            )

            if ready:
                break

            if time.time() > deadline:
                self.fail(
                    "flight/EGO interface stack not ready"
                )

            rospy.sleep(0.1)

        print(
            GREEN +
            "[ OK ] flight + EGO interfaces ready" +
            RESET
        )


    def wait_manual_arm(self):

        print()
        print("----------------------------------------")
        print(" WAITING FOR PILOT")
        print(" CH7: LOW -> MID -> HIGH")
        print(" CH6: HIGH / POSCTL")
        print(" CH5: ARM")
        print(" No software ARM is used.")
        print("----------------------------------------")
        print(flush=True)

        while not rospy.is_shutdown():

            if (
                self.state is not None
                and self.state.armed
                and self.mission_allowed
                and self.state.mode in [
                    "POSCTL",
                    "OFFBOARD"
                ]
            ):
                break

            rospy.sleep(0.1)

        print(
            GREEN +
            "[ OK ] PILOT AUTHORIZATION RECEIVED" +
            RESET
        )


    def run_flight_ops(self, command):

        script = os.path.expanduser(
            "~/contest2_uav/runtime/"
            "ego_uav_ws/src/ego_uav_interface/scripts/"
            "uav2_flight_ops_test.py"
        )

        print()
        print(
            CYAN +
            "[ FLIGHT OPS ] %s" % command.upper() +
            RESET,
            flush=True
        )

        ret = subprocess.call([
            script,
            command
        ])

        if ret != 0:
            self.fail(
                "uav2_flight_ops_test.py %s failed"
                % command
            )


    def wait_takeoff_settle(
        self,
        ground_z,
        timeout=15.0
    ):

        target_z = (
            ground_z
            + self.takeoff_height
        )

        stable_since = None
        deadline = time.time() + timeout
        last_print = 0.0

        print()
        print(
            CYAN +
            "[ TAKEOFF SETTLE ] target_z=%.3f"
            % target_z +
            RESET
        )

        while not rospy.is_shutdown():

            if self.odom is None:
                rospy.sleep(0.02)
                continue

            if not self.flight_healthy:
                self.fail(
                    "flight_healthy lost during takeoff settle"
                )

            if (
                self.state is None
                or not self.state.armed
                or self.state.mode != "OFFBOARD"
            ):
                self.fail(
                    "PX4 left normal OFFBOARD state "
                    "during takeoff settle"
                )

            z = self.odom.pose.pose.position.z
            vz = self.odom.twist.twist.linear.z

            ez = z - target_z

            physically_ready = (
                self.point_status == "AIRBORNE_HOLD"
                and abs(ez) <= self.altitude_tolerance
                and abs(vz) <= 0.10
            )

            now = time.time()

            if now - last_print >= 0.20:

                print(
                    "\r\033[2K"
                    "SETTLE   z=%6.3f target=%6.3f "
                    "err=%+5.3f vz=%+5.2f  point=%s"
                    %
                    (
                        z,
                        target_z,
                        ez,
                        vz,
                        self.point_status
                    ),
                    end="",
                    flush=True
                )

                last_print = now

            if physically_ready:

                if stable_since is None:
                    stable_since = now

                elif now - stable_since >= 0.60:

                    print()

                    print(
                        GREEN +
                        "[ OK ] TAKEOFF PHYSICALLY SETTLED "
                        "z=%.3f" % z +
                        RESET
                    )

                    return

            else:
                stable_since = None

            if now > deadline:

                print()

                self.fail(
                    "takeoff physical settle timeout"
                )

            rospy.sleep(0.02)


    def body_to_world(
        self,
        home_x,
        home_y,
        forward,
        left,
        yaw0
    ):

        dx = (
            forward * math.cos(yaw0)
            -
            left * math.sin(yaw0)
        )

        dy = (
            forward * math.sin(yaw0)
            +
            left * math.cos(yaw0)
        )

        return (
            home_x + dx,
            home_y + dy
        )


    def wait_executor_idle(
        self,
        timeout=2.0
    ):

        deadline = time.time() + timeout

        while not rospy.is_shutdown():

            if self.rviz_status == "IDLE":
                return

            if time.time() > deadline:
                self.fail(
                    "rviz_goal_executor did not return to IDLE"
                )

            rospy.sleep(0.02)


    def execute_ego_waypoint(
        self,
        label,
        gx,
        gy
    ):

        if self.odom is None:
            self.fail("no odometry before " + label)

        z = self.odom.pose.pose.position.z
        yaw_now = self.yaw()

        # Same conservative area already used by the proven
        # single-goal demo.
        if not (
            -2.8 <= gx <= 3.3
            and
            -1.5 <= gy <= 1.5
        ):
            self.fail(
                "%s outside EGO safe area: "
                "(%.3f, %.3f)"
                %
                (
                    label,
                    gx,
                    gy
                )
            )

        print()
        print("----------------------------------------")
        print(
            " %s -> (%.3f, %.3f, %.3f)"
            %
            (
                label,
                gx,
                gy,
                z
            )
        )
        print("----------------------------------------")

        # Reassert HOLD before every planning/execution cycle.
        # This does not command a turn; it captures the
        # aircraft's current heading for the next EGO leg.
        self.yaw_mode_pub.publish(
            String(data="HOLD")
        )

        rospy.sleep(0.20)

        self.ego_started = False
        self.ego_done = False
        self.ego_abort = None

        msg = PoseStamped()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"

        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.position.z = z

        msg.pose.orientation.z = math.sin(
            yaw_now / 2.0
        )

        msg.pose.orientation.w = math.cos(
            yaw_now / 2.0
        )

        self.goal_pub.publish(msg)

        print(
            CYAN +
            "[ EGO GOAL SENT ] %s" % label +
            RESET
        )

        deadline = time.time() + 45.0

        while not rospy.is_shutdown():

            if self.ego_abort is not None:
                self.fail(
                    "%s: %s"
                    %
                    (
                        label,
                        self.ego_abort
                    )
                )

            if self.ego_done:
                break

            if not self.flight_healthy:
                self.fail(
                    "flight_healthy lost during " + label
                )

            if not self.mission_allowed:
                self.fail(
                    "CH7 mission permission lost during "
                    + label
                )

            if (
                self.state is None
                or not self.state.armed
            ):
                self.fail(
                    "vehicle disarmed during " + label
                )

            if self.state.mode != "OFFBOARD":
                self.fail(
                    "PX4 left OFFBOARD during " + label
                )

            if time.time() > deadline:
                self.fail(
                    "%s EGO timeout" % label
                )

            rospy.sleep(0.05)

        p = self.odom.pose.pose.position

        err = math.hypot(
            p.x - gx,
            p.y - gy
        )

        print(
            GREEN +
            "[ WAYPOINT REACHED ] %s "
            "actual=(%.3f, %.3f, %.3f) "
            "xy_err=%.3f m yaw=%.1f deg"
            %
            (
                label,
                p.x,
                p.y,
                p.z,
                err,
                math.degrees(self.yaw())
            )
            + RESET
        )

        # Executor publishes IDLE after finishing a waypoint.
        self.wait_executor_idle()

        rospy.sleep(
            self.waypoint_pause
        )


    def execute_rectangle(
        self,
        home_x,
        home_y,
        yaw0
    ):

        hf = self.half_forward
        hl = self.half_left

        # Same route as the frozen POINT rectangle:
        #
        # P1 = backward + left
        # P2 = forward  + left
        # P3 = forward  + right
        # P4 = backward + right
        # HOME
        body_points = [
            ("P1", -hf, +hl),
            ("P2", +hf, +hl),
            ("P3", +hf, -hl),
            ("P4", -hf, -hl),
            ("HOME", 0.0, 0.0),
        ]

        world_points = []

        for label, forward, left in body_points:

            x, y = self.body_to_world(
                home_x,
                home_y,
                forward,
                left,
                yaw0
            )

            world_points.append(
                (
                    label,
                    x,
                    y
                )
            )

        print()
        print("========================================")
        print(" EGO RECTANGLE ROUTE")
        print(" yaw remains HOLD")
        print("========================================")

        for label, x, y in world_points:
            print(
                "%-4s : (% .3f, % .3f)"
                %
                (
                    label,
                    x,
                    y
                )
            )

        for label, x, y in world_points:

            self.execute_ego_waypoint(
                label,
                x,
                y
            )

        print()
        print("========================================")
        print(" EGO RECTANGLE COMPLETE")
        print(" Returned to takeoff XY")
        print("========================================")


    def run(self):

        self.wait_stack()

        # Capture takeoff reference on the ground.
        p = self.odom.pose.pose.position

        home_x = p.x
        home_y = p.y
        ground_z = p.z

        rospy.set_param(
            "/uav2_touchdown/ground_z_reference",
            ground_z
        )

        print(
            "[ HOME ] x=%.3f y=%.3f ground_z=%.3f"
            %
            (
                home_x,
                home_y,
                ground_z
            )
        )

        self.wait_manual_arm()

        print()
        print("========================================")
        print(" TAKEOFF")
        print("========================================")

        self.run_flight_ops("takeoff")

        self.wait_takeoff_settle(
            ground_z
        )

        # Initial body frame for the whole rectangle.
        yaw0 = self.yaw()

        print(
            "[ RECTANGLE FRAME ] initial yaw = %.1f deg"
            % math.degrees(yaw0)
        )

        self.execute_rectangle(
            home_x,
            home_y,
            yaw0
        )

        print()
        print(
            "Hovering at HOME for %.1f s ..."
            % self.hover_time
        )

        rospy.sleep(
            self.hover_time
        )

        print()
        print("========================================")
        print(" LANDING")
        print("========================================")

        self.run_flight_ops("land")

        print()
        print("========================================")
        print(" UAV2 EGO RECTANGLE DEMO COMPLETED")
        print("========================================")


if __name__ == "__main__":

    rospy.init_node(
        "uav2_ego_rectangle"
    )

    demo = EgoSingleGoalDemo()

    try:
        demo.run()

    except RuntimeError:
        pass
