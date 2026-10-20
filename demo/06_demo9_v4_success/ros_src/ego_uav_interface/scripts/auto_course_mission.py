#!/usr/bin/env python3

import math
import os
import re
import subprocess
import threading
import time

import rospy
import rosnode

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from std_msgs.msg import Bool, String, Float64
from tf.transformations import quaternion_from_euler


RESET = "\033[0m"
GREEN = "\033[1;32m"
CYAN = "\033[1;36m"
YELLOW = "\033[1;33m"
RED = "\033[1;31m"
BOLD = "\033[1m"


def wrap_angle(a):
    return math.atan2(
        math.sin(a),
        math.cos(a)
    )


class AutoCourseMission:

    def __init__(self):

        # ====================================================
        # Mission geometry
        # ====================================================

        self.p1_x = rospy.get_param(
            "~p1_x", 2.5
        )

        self.p1_y = rospy.get_param(
            "~p1_y", -0.5
        )

        self.p1_yaw = math.radians(
            rospy.get_param(
                "~p1_yaw_deg", 90.0
            )
        )

        self.p2_x = rospy.get_param(
            "~p2_x", -2.2
        )

        self.p2_y = rospy.get_param(
            "~p2_y", 0.5
        )

        self.p2_yaw = math.radians(
            rospy.get_param(
                "~p2_yaw_deg", 180.0
            )
        )

        self.hover_time = rospy.get_param(
            "~hover_time", 3.0
        )

        self.yaw_tol = math.radians(
            rospy.get_param(
                "~yaw_tolerance_deg", 8.0
            )
        )

        self.yaw_stable_time = rospy.get_param(
            "~yaw_stable_time", 0.8
        )

        self.yaw_timeout = rospy.get_param(
            "~yaw_timeout", 15.0
        )

        # ====================================================
        # State
        # ====================================================

        self.state = None
        self.odom = None

        self.flight_healthy = False
        self.stream_ready = False
        self.mission_allowed = False

        self.lock = threading.Lock()

        self.status_seq = 0

        self.translate_seq = -1
        self.translate_data = ""

        self.done_seq = -1
        self.done_data = ""

        self.abort_seq = -1
        self.abort_data = ""

        self.idle_seq = -1
        self.running_data = ""

        # ====================================================
        # Publishers
        # ====================================================

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

        self.yaw_target_pub = rospy.Publisher(
            "/ego_uav_interface/yaw_target_deg",
            Float64,
            queue_size=1,
            latch=True
        )

        # ====================================================
        # Subscribers
        # ====================================================

        rospy.Subscriber(
            "/uav1/mavros/state",
            State,
            self.state_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/uav1/mavros/local_position/odom",
            Odometry,
            self.odom_cb,
            queue_size=30
        )

        rospy.Subscriber(
            "/ego_uav_interface/flight_healthy",
            Bool,
            self.health_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/ego_uav_interface/offboard_stream_ready",
            Bool,
            self.stream_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/ego_uav_interface/rc_mission_allowed",
            Bool,
            self.mission_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/ego_uav_interface/rviz_goal_status",
            String,
            self.executor_status_cb,
            queue_size=100
        )


    # ========================================================
    # Callbacks
    # ========================================================

    def state_cb(self, msg):
        self.state = msg


    def odom_cb(self, msg):
        self.odom = msg


    def health_cb(self, msg):
        self.flight_healthy = msg.data


    def stream_cb(self, msg):
        self.stream_ready = msg.data


    def mission_cb(self, msg):
        self.mission_allowed = msg.data


    def executor_status_cb(self, msg):

        s = msg.data.strip()

        with self.lock:

            self.status_seq += 1
            seq = self.status_seq

            if s.startswith("RUNNING "):
                self.running_data = s

            elif re.match(
                r"^TRANSLATE\s+traj_id=\d+$",
                s
            ):
                self.translate_seq = seq
                self.translate_data = s

            elif s.startswith("DONE"):

                self.done_seq = seq
                self.done_data = s

            elif s.startswith("ABORT"):

                self.abort_seq = seq
                self.abort_data = s

            elif s == "IDLE":

                self.idle_seq = seq


    # ========================================================
    # Utility
    # ========================================================

    def banner(self, text):

        print()
        print(
            BOLD +
            "========================================"
            + RESET
        )

        print(
            BOLD +
            " " + text
            + RESET
        )

        print(
            BOLD +
            "========================================"
            + RESET,
            flush=True
        )


    def fail(self, text):

        print()
        print(
            RED +
            "MISSION ABORT: " +
            text +
            RESET,
            flush=True
        )

        raise RuntimeError(text)


    def current_yaw(self):

        if self.odom is None:
            return None

        q = self.odom.pose.pose.orientation

        siny = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        return math.atan2(
            siny,
            cosy
        )


    # ========================================================
    # Startup
    # ========================================================

    def wait_stack(self):

        self.banner(
            "AUTO COURSE INITIALIZING"
        )

        required = [
            "/ego_trajectory_bridge",
            "/rviz_goal_executor"
        ]

        deadline = time.time() + 30.0

        while not rospy.is_shutdown():

            try:
                nodes = rosnode.get_node_names()
            except Exception:
                nodes = []

            missing = [
                n for n in required
                if n not in nodes
            ]

            if not missing:
                break

            if time.time() > deadline:

                self.fail(
                    "missing nodes: " +
                    ", ".join(missing)
                )

            rospy.sleep(0.2)

        deadline = time.time() + 20.0

        while not rospy.is_shutdown():

            if (
                self.odom is not None
                and self.state is not None
                and self.flight_healthy
                and self.stream_ready
            ):
                break

            if time.time() > deadline:

                self.fail(
                    "flight stack not ready"
                )

            rospy.sleep(0.1)

        print(
            GREEN +
            "[ OK ] flight stack ready"
            + RESET
        )


    def wait_manual_arm(self):

        print()
        print("----------------------------------------")
        print(" WAITING FOR PILOT")
        print(" CH7: LOW -> MID -> HIGH")
        print(" CH6: HIGH / POSCTL")
        print(" CH5: ARM")
        print(" Mission starts automatically after ARM.")
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
            "[ OK ] PILOT AUTHORIZATION RECEIVED"
            + RESET
        )


    # ========================================================
    # Proven takeoff / landing module
    # ========================================================

    def run_flight_ops(self, command):

        script = os.path.expanduser(
            "/home/yundrone/contest2_uav/runtime/ego_uav_ws/src/ego_uav_interface/scripts/flight_ops_test.py"
        )

        print()
        print(
            CYAN +
            "[ FLIGHT OPS ] %s"
            % command.upper()
            + RESET,
            flush=True
        )

        ret = subprocess.call([
            script,
            command
        ])

        if ret != 0:

            self.fail(
                "flight_ops_test.py %s failed"
                % command
            )


    def wait_takeoff_settle(
        self,
        ground_z,
        timeout=7.0
    ):

        # flight_ops reports AIRBORNE_HOLD when the planned
        # trajectory has completed. The real vehicle can still
        # be converging toward the final altitude.
        #
        # Before handing control to EGO require:
        #   actual climb >= 0.60 m
        #   |vz| <= 0.10 m/s
        #   stable for 0.60 s

        min_climb = 0.60
        vz_limit = 0.10
        stable_required = 0.60

        deadline = time.time() + timeout
        stable_since = None
        last_print = 0.0

        print()
        print(
            CYAN +
            "[ TAKEOFF SETTLE ] waiting for physical altitude..."
            + RESET
        )

        while not rospy.is_shutdown():

            if self.odom is None:
                rospy.sleep(0.02)
                continue

            z = self.odom.pose.pose.position.z
            vz = self.odom.twist.twist.linear.z
            dz = z - ground_z

            now = time.time()

            if now - last_print >= 0.15:

                print(
                    "\r\033[2K"
                    "SETTLE   z=%6.3f  dz=%5.3f  vz=%+5.2f m/s"
                    %
                    (
                        z,
                        dz,
                        vz
                    ),
                    end="",
                    flush=True
                )

                last_print = now

            good = (
                dz >= min_climb
                and abs(vz) <= vz_limit
            )

            if good:

                if stable_since is None:
                    stable_since = now

                elif (
                    now - stable_since
                    >= stable_required
                ):

                    print()

                    print(
                        GREEN +
                        "[ OK ] TAKEOFF PHYSICALLY SETTLED "
                        "z=%.3f dz=%.3f"
                        %
                        (
                            z,
                            dz
                        )
                        + RESET
                    )

                    return

            else:
                stable_since = None

            if (
                self.state is None
                or not self.state.armed
            ):

                print()

                self.fail(
                    "vehicle disarmed while waiting for takeoff settle"
                )

            if self.state.mode != "OFFBOARD":

                print()

                self.fail(
                    "PX4 left OFFBOARD while waiting for takeoff settle"
                )

            if now > deadline:

                print()

                self.fail(
                    "takeoff did not physically settle: "
                    "z=%.3f dz=%.3f vz=%.3f"
                    %
                    (
                        z,
                        dz,
                        vz
                    )
                )

            rospy.sleep(0.02)


    def wait_final_disarm(
        self,
        timeout=5.0
    ):

        deadline = time.time() + timeout

        while not rospy.is_shutdown():

            if (
                self.state is not None
                and not self.state.armed
            ):

                print(
                    GREEN +
                    "[ OK ] PX4 DISARMED"
                    + RESET
                )

                return

            if time.time() > deadline:

                print(
                    YELLOW +
                    "[ WARN ] Aircraft is on ground "
                    "but PX4 remains armed."
                    + RESET
                )

                print(
                    YELLOW +
                    "Use CH5 to DISARM."
                    + RESET
                )

                return

            rospy.sleep(0.10)


    # ========================================================
    # Yaw modes
    # ========================================================

    def yaw_hold(self):

        self.yaw_mode_pub.publish(
            String(data="HOLD")
        )

        rospy.sleep(0.15)

        yaw = self.current_yaw()

        if yaw is not None:

            print(
                CYAN +
                "[ YAW ] HOLD %.1f deg"
                % math.degrees(yaw)
                + RESET
            )


    def yaw_tangent(self):

        self.yaw_mode_pub.publish(
            String(data="TANGENT")
        )

        rospy.sleep(0.15)

        print(
            CYAN +
            "[ YAW ] TANGENT MODE"
            + RESET
        )


    def rotate_to_yaw(
        self,
        target_deg,
        label
    ):

        target = math.radians(
            target_deg
        )

        yaw0 = self.current_yaw()

        if yaw0 is None:
            self.fail(
                "no yaw before rotation"
            )

        initial_error = abs(
            wrap_angle(
                target - yaw0
            )
        )

        initial_error = max(
            initial_error,
            math.radians(1.0)
        )

        # Publish target first, then enter FIXED mode.
        self.yaw_target_pub.publish(
            Float64(data=target_deg)
        )

        rospy.sleep(0.05)

        self.yaw_mode_pub.publish(
            String(data="FIXED")
        )

        print()
        print(
            CYAN +
            "[ YAW ROTATE ] %s -> %.1f deg"
            % (
                label,
                target_deg
            )
            + RESET
        )

        deadline = (
            time.time()
            +
            self.yaw_timeout
        )

        stable_since = None
        last_print = 0.0

        while not rospy.is_shutdown():

            yaw = self.current_yaw()

            if yaw is None:
                rospy.sleep(0.05)
                continue

            err = abs(
                wrap_angle(
                    target - yaw
                )
            )

            progress = (
                1.0 -
                min(
                    1.0,
                    err / initial_error
                )
            ) * 100.0

            now = time.time()

            if now - last_print > 0.20:

                print(
                    "\r\033[2K"
                    +
                    "[YAW] actual=%6.1f deg  "
                    "target=%6.1f deg  "
                    "progress=%5.1f%%"
                    %
                    (
                        math.degrees(yaw),
                        target_deg,
                        progress
                    ),
                    end="",
                    flush=True
                )

                last_print = now

            if err <= self.yaw_tol:

                if stable_since is None:

                    stable_since = time.time()

                elif (
                    time.time()
                    -
                    stable_since
                    >=
                    self.yaw_stable_time
                ):

                    print()
                    print(
                        GREEN +
                        "[ YAW OK ] %s actual=%.1f deg"
                        % (
                            label,
                            math.degrees(yaw)
                        )
                        + RESET
                    )

                    return

            else:

                stable_since = None

            if time.time() > deadline:

                print()

                self.fail(
                    "%s yaw timeout, actual=%.1f"
                    %
                    (
                        label,
                        math.degrees(yaw)
                    )
                )

            rospy.sleep(0.05)


    # ========================================================
    # EGO goal
    # ========================================================

    def execute_goal(
        self,
        x,
        y,
        display_yaw_deg,
        label,
        show_progress=False,
        timeout=90.0
    ):

        if self.odom is None:
            self.fail(
                "no odometry"
            )

        z = self.odom.pose.pose.position.z

        sx = self.odom.pose.pose.position.x
        sy = self.odom.pose.pose.position.y

        initial_distance = math.hypot(
            x - sx,
            y - sy
        )

        initial_distance = max(
            initial_distance,
            0.01
        )

        deadline_connect = (
            time.time() + 5.0
        )

        while (
            self.goal_pub.get_num_connections()
            == 0
        ):

            if time.time() > deadline_connect:

                self.fail(
                    "rviz_goal_executor not connected"
                )

            rospy.sleep(0.05)

        msg = PoseStamped()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        q = quaternion_from_euler(
            0.0,
            0.0,
            math.radians(
                display_yaw_deg
            )
        )

        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        with self.lock:
            baseline = self.status_seq

        print()
        print(
            CYAN +
            ">>> %s : (%.2f, %.2f, %.2f)"
            %
            (
                label,
                x,
                y,
                z
            )
            + RESET
        )

        self.goal_pub.publish(msg)

        started = False
        last_progress_print = 0.0

        deadline = time.time() + timeout

        while not rospy.is_shutdown():

            with self.lock:

                translate_seq = (
                    self.translate_seq
                )

                translate_data = (
                    self.translate_data
                )

                done_seq = (
                    self.done_seq
                )

                abort_seq = (
                    self.abort_seq
                )

                abort_data = (
                    self.abort_data
                )

            if abort_seq > baseline:

                print()

                self.fail(
                    "%s -> %s"
                    %
                    (
                        label,
                        abort_data
                    )
                )

            if (
                translate_seq > baseline
                and not started
            ):

                started = True

                print(
                    GREEN +
                    "[ TRAJECTORY START ] "
                    + label
                    + " | "
                    + translate_data
                    + RESET
                )

            # =================================================
            # P2 terminal display
            # =================================================

            if (
                show_progress
                and started
                and self.odom is not None
            ):

                now = time.time()

                if (
                    now -
                    last_progress_print
                    >=
                    0.25
                ):

                    p = (
                        self.odom.pose.pose.position
                    )

                    remaining = math.hypot(
                        x - p.x,
                        y - p.y
                    )

                    progress = (
                        1.0
                        -
                        remaining
                        /
                        initial_distance
                    ) * 100.0

                    progress = max(
                        0.0,
                        min(
                            100.0,
                            progress
                        )
                    )

                    yaw = self.current_yaw()

                    yaw_deg = (
                        math.degrees(yaw)
                        if yaw is not None
                        else float("nan")
                    )

                    print(
                        "\r\033[2K"
                        +
                        "[P2] "
                        "pos=(%6.2f,%6.2f,%5.2f)  "
                        "remain=%5.2f m  "
                        "progress=%5.1f%%  "
                        "yaw=%6.1f deg"
                        %
                        (
                            p.x,
                            p.y,
                            p.z,
                            remaining,
                            progress,
                            yaw_deg
                        ),
                        end="",
                        flush=True
                    )

                    last_progress_print = now

            if done_seq > baseline:

                if show_progress:
                    print()

                p = self.odom.pose.pose.position

                print(
                    GREEN +
                    "[ GOAL REACHED ] %s "
                    "actual=(%.2f,%.2f,%.2f)"
                    %
                    (
                        label,
                        p.x,
                        p.y,
                        p.z
                    )
                    + RESET
                )

                # Wait until executor fully leaves BUSY state.
                idle_deadline = (
                    time.time() + 3.0
                )

                while not rospy.is_shutdown():

                    with self.lock:
                        idle_seq = self.idle_seq

                    if idle_seq > done_seq:
                        break

                    if (
                        time.time()
                        >
                        idle_deadline
                    ):

                        self.fail(
                            label +
                            " DONE but executor not IDLE"
                        )

                    rospy.sleep(0.02)

                rospy.sleep(0.10)

                return

            if (
                self.state is None
                or not self.state.armed
            ):

                if show_progress:
                    print()

                self.fail(
                    "vehicle disarmed during "
                    + label
                )

            if self.state.mode != "OFFBOARD":

                if show_progress:
                    print()

                self.fail(
                    "PX4 left OFFBOARD during %s, current mode=%s"
                    %
                    (
                        label,
                        self.state.mode
                    )
                )

            if time.time() > deadline:

                if show_progress:
                    print()

                self.fail(
                    label +
                    " timeout"
                )

            rospy.sleep(0.05)


    # ========================================================
    # Main mission
    # ========================================================

    def run(self):

        self.wait_stack()

        self.banner(
            "MISSION ROUTE"
        )

        print(
            "Takeoff"
        )

        print(
            "P1 : (%.2f, %.2f)"
            % (
                self.p1_x,
                self.p1_y
            )
        )

        print(
            "P1 arrival -> plan P2 -> align planned tangent"
        )

        print(
            "P2 : (%.2f, %.2f)"
            % (
                self.p2_x,
                self.p2_y
            )
        )

        print(
            "P2 flight -> tangent yaw"
        )

        print(
            "P2 arrival -> rotate to 180 deg"
        )

        print(
            "Hover %.1f s -> land"
            %
            self.hover_time
        )

        self.wait_manual_arm()

        # ====================================================
        # Takeoff
        # ====================================================

        self.banner(
            "TAKEOFF"
        )

        if self.odom is None:
            self.fail(
                "no odometry before takeoff"
            )

        ground_z = (
            self.odom.pose.pose.position.z
        )

        self.run_flight_ops(
            "takeoff"
        )

        self.wait_takeoff_settle(
            ground_z
        )

        print(
            GREEN +
            "[ OK ] mission altitude = %.3f m"
            %
            self.odom.pose.pose.position.z
            + RESET
        )

        # ====================================================
        # P1
        #
        # Do NOT follow trajectory tangent.
        # Keep the heading the vehicle already has.
        # ====================================================

        self.banner(
            "LEG 1 - POSITION ONLY"
        )

        self.yaw_hold()

        self.execute_goal(
            self.p1_x,
            self.p1_y,
            90.0,
            "P1",
            show_progress=False
        )

        # Now rotate in place.
        print(
            CYAN +
            "[ P1 ] position reached; "
            "P2 will align to planned tangent"
            + RESET
        )

        self.banner(
            "P1 REACHED: (%.1f,%.1f), YAW=90 DEG"
            %
            (
                self.p1_x,
                self.p1_y
            )
        )

        # ====================================================
        # P2
        #
        # Main course:
        # trajectory tangent yaw is enabled here.
        # This is the leg we will later use for obstacle
        # avoidance.
        # ====================================================

        self.banner(
            "LEG 2 - MAIN COURSE"
        )

        self.yaw_tangent()

        self.execute_goal(
            self.p2_x,
            self.p2_y,
            180.0,
            "P2",
            show_progress=True
        )

        # No second tiny EGO trajectory.
        # We are already at the P2 tolerance region.
        # Rotate in place to exact terminal yaw.
        self.rotate_to_yaw(
            180.0,
            "P2"
        )

        # Keep final yaw during hover.
        self.yaw_target_pub.publish(
            Float64(data=180.0)
        )

        self.yaw_mode_pub.publish(
            String(data="FIXED")
        )

        p = self.odom.pose.pose.position
        yaw = self.current_yaw()

        self.banner(
            "TARGET REACHED"
        )

        print(
            GREEN +
            "Final position : "
            "(%.3f, %.3f, %.3f)"
            %
            (
                p.x,
                p.y,
                p.z
            )
            + RESET
        )

        print(
            GREEN +
            "Final yaw      : %.1f deg"
            %
            math.degrees(yaw)
            + RESET
        )

        # ====================================================
        # Hover 3 s
        # ====================================================

        print()
        print(
            CYAN +
            "Hovering for %.1f s ..."
            %
            self.hover_time
            + RESET
        )

        end_time = (
            time.time()
            +
            self.hover_time
        )

        while (
            time.time() < end_time
            and not rospy.is_shutdown()
        ):

            if (
                self.state is None
                or not self.state.armed
                or self.state.mode != "OFFBOARD"
            ):

                self.fail(
                    "flight state lost during hover"
                )

            rospy.sleep(0.1)

        # ====================================================
        # Land
        # ====================================================

        self.banner(
            "LANDING"
        )

        self.run_flight_ops(
            "land"
        )

        self.wait_final_disarm()

        self.banner(
            "AUTO COURSE COMPLETED"
        )


if __name__ == "__main__":

    rospy.init_node(
        "auto_course_mission"
    )

    node = AutoCourseMission()

    try:

        node.run()

    except Exception as e:

        print()
        print(
            RED +
            "MISSION STOPPED: %s"
            %
            str(e)
            + RESET,
            flush=True
        )

        # Do not auto-arm, auto-kill or issue an emergency
        # landing here.  Existing final EGO hold remains
        # available and CH6 is still the hardware takeover.
        raise
