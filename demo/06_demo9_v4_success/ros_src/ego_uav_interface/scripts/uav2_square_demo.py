#!/usr/bin/env python3

import math
import os
import subprocess
import time

import rospy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


GREEN = "\033[32m"
CYAN = "\033[36m"
YELLOW = "\033[33m"
RED = "\033[31m"
RESET = "\033[0m"


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class UAV2SquareDemo:

    def __init__(self):

        self.state = None
        self.odom = None

        self.health = False
        self.stream = False
        self.mission_allowed = False

        self.point_status = None
        self.mux_status = None

        self.offset = rospy.get_param(
            "~offset",
            0.50
        )

        # Must match the frozen POINT takeoff height.
        # All waypoint motion stays on this HOME-relative plane.
        self.takeoff_height = rospy.get_param(
            "~takeoff_height",
            0.80
        )

        self.altitude_tolerance = rospy.get_param(
            "~altitude_tolerance",
            0.04
        )

        self.arrival_tol = rospy.get_param(
            "~arrival_tol",
            0.08
        )

        self.speed_tol = rospy.get_param(
            "~speed_tol",
            0.10
        )

        self.arrival_stable_time = rospy.get_param(
            "~arrival_stable_time",
            0.60
        )

        self.waypoint_hold_time = rospy.get_param(
            "~waypoint_hold_time",
            1.0
        )

        self.waypoint_timeout = rospy.get_param(
            "~waypoint_timeout",
            15.0
        )

        self.yaw_abort_deg = rospy.get_param(
            "~yaw_abort_deg",
            15.0
        )

        self.flight_ops = os.path.expanduser(
            "~/contest2_uav/runtime/"
            "ego_uav_ws/src/ego_uav_interface/scripts/"
            "uav2_flight_ops_test.py"
        )

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

        self.goal_pub = rospy.Publisher(
            "/ego_uav_interface/goal",
            PoseStamped,
            queue_size=5
        )

    # ============================================================
    # Callbacks
    # ============================================================

    def state_cb(self, msg):
        self.state = msg

    def odom_cb(self, msg):
        self.odom = msg

    def health_cb(self, msg):
        self.health = msg.data

    def stream_cb(self, msg):
        self.stream = msg.data

    def mission_cb(self, msg):
        self.mission_allowed = msg.data

    def point_cb(self, msg):
        self.point_status = msg.data

    def mux_cb(self, msg):
        self.mux_status = msg.data

    # ============================================================
    # State helpers
    # ============================================================

    def position(self):
        p = self.odom.pose.pose.position
        return p.x, p.y, p.z

    def yaw(self):
        return yaw_from_quaternion(
            self.odom.pose.pose.orientation
        )

    def speed(self):
        v = self.odom.twist.twist.linear
        return math.sqrt(
            v.x * v.x +
            v.y * v.y +
            v.z * v.z
        )

    def fail(self, text):

        print()
        print(
            RED +
            "MISSION STOPPED: " +
            text +
            RESET
        )

        print(
            "CH7 LOW remains the autonomy takeover path."
        )

        raise RuntimeError(text)

    # ============================================================
    # Startup
    # ============================================================

    def wait_stack(self):

        print()
        print("========================================")
        print(" UAV2 FOUR-WAYPOINT DEMO INITIALIZING")
        print("========================================")

        deadline = time.time() + 20.0

        while not rospy.is_shutdown():

            ready = (
                self.state is not None
                and self.state.connected
                and self.odom is not None
                and self.health
                and self.stream
                and self.point_status is not None
                and self.mux_status is not None
            )

            if ready:
                break

            if time.time() > deadline:
                self.fail("flight stack not ready")

            rospy.sleep(0.1)

        print(
            GREEN +
            "[ OK ] flight stack ready"
            + RESET
        )

        print()
        print("NO AUTOMATIC ARM")
        print("NO YAW ROTATION")
        print("POINT waypoint mode")
        print()

    def wait_manual_arm(self):

        print("----------------------------------------")
        print(" WAITING FOR PILOT")
        print(" CH7: LOW -> MID -> HIGH")
        print(" CH6: HIGH / POSCTL")
        print(" CH5: ARM")
        print(" Mission starts automatically after ARM.")
        print("----------------------------------------")

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

    # ============================================================
    # Frozen flight operations
    # ============================================================

    def run_flight_ops(self, command):

        print()
        print(
            CYAN +
            "[ FLIGHT OPS ] %s"
            % command.upper()
            + RESET
        )

        ret = subprocess.call([
            self.flight_ops,
            command
        ])

        if ret != 0:
            self.fail(
                "uav2_flight_ops_test.py %s failed"
                % command
            )

    def wait_takeoff_settle(
        self,
        ground_z
    ):

        print()
        print(
            CYAN +
            "[ TAKEOFF SETTLE ] "
            "waiting for physical altitude..."
            + RESET
        )

        deadline = time.time() + 15.0
        stable_since = None
        last_print = 0.0

        while not rospy.is_shutdown():

            if self.state is None or self.odom is None:
                self.fail("state/odom lost")

            if not self.state.armed:
                self.fail("PX4 disarmed during takeoff")

            if self.state.mode != "OFFBOARD":
                self.fail(
                    "PX4 left OFFBOARD during takeoff settle"
                )

            if not self.health:
                self.fail(
                    "flight health lost during takeoff settle"
                )

            _, _, z = self.position()
            vz = self.odom.twist.twist.linear.z

            dz = z - ground_z

            target_z = (
                ground_z
                +
                self.takeoff_height
            )

            altitude_error = (
                z - target_z
            )

            physically_ready = (
                abs(altitude_error)
                    <= self.altitude_tolerance
                and abs(vz) <= 0.10
            )

            now = time.time()

            if physically_ready:

                if stable_since is None:
                    stable_since = now

                if now - stable_since >= 0.60:

                    print(
                        GREEN +
                        "[ OK ] TAKEOFF PHYSICALLY SETTLED "
                        "z=%.3f dz=%.3f"
                        % (z, dz)
                        + RESET
                    )

                    return

            else:
                stable_since = None

            if now - last_print >= 0.5:

                print(
                    "SETTLE z=% .3f target=%.3f "
                    "err=%+.3f vz=%+.2f m/s"
                    % (
                        z,
                        target_z,
                        altitude_error,
                        vz
                    )
                )

                last_print = now

            if now > deadline:
                self.fail("takeoff settle timeout")

            rospy.sleep(0.05)

    # ============================================================
    # Waypoints
    # ============================================================

    def check_airborne(self):

        if self.state is None or self.odom is None:
            self.fail("state/odom unavailable")

        if not self.state.armed:
            self.fail("PX4 unexpectedly disarmed")

        if self.state.mode != "OFFBOARD":
            self.fail("PX4 left OFFBOARD")

        if not self.health:
            self.fail("flight health lost")

        if not self.mission_allowed:
            self.fail("mission permission removed")

        # POINT_STALE is expected after a completed POINT
        # trajectory. At that time point_traj_node is already in
        # AIRBORNE_HOLD and trajectory_interface maintains its
        # own HOLD fallback. Only an unexpected EGO/unknown mux
        # state is considered a mission fault.
        if self.mux_status not in [
            "POINT_ACTIVE",
            "POINT_STALE"
        ]:
            self.fail(
                "unexpected trajectory mux state: %s"
                % str(self.mux_status)
            )

    def body_to_world(
        self,
        home_x,
        home_y,
        yaw0,
        forward,
        left
    ):

        # FLU:
        #   +forward = body X
        #   +left    = body Y
        #
        # Converted into the local ENU/map frame.

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

    def publish_goal(
        self,
        x,
        y,
        z,
        yaw0
    ):

        msg = PoseStamped()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        # Fixed initial yaw for every waypoint.
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(
            0.5 * yaw0
        )
        msg.pose.orientation.w = math.cos(
            0.5 * yaw0
        )

        self.goal_pub.publish(msg)

    def fly_to(
        self,
        name,
        x,
        y,
        z,
        yaw0
    ):

        self.check_airborne()

        while (
            self.goal_pub.get_num_connections() == 0
            and not rospy.is_shutdown()
        ):
            rospy.sleep(0.05)

        print()
        print("----------------------------------------")
        print(
            " %s -> (%.3f, %.3f, %.3f)"
            % (
                name,
                x,
                y,
                z
            )
        )
        print("----------------------------------------")

        self.publish_goal(
            x,
            y,
            z,
            yaw0
        )

        # Explicitly confirm that point_traj_node accepted the
        # new goal and that the POINT source became live again.
        start_deadline = time.time() + 2.0

        while not rospy.is_shutdown():

            self.check_airborne()

            if (
                self.point_status == "POINT_TRAJECTORY"
                and self.mux_status == "POINT_ACTIVE"
            ):
                print(
                    GREEN +
                    "[ POINT STARTED ] %s"
                    % name
                    + RESET
                )
                break

            if time.time() > start_deadline:
                self.fail(
                    "%s POINT start handshake timeout: "
                    "point=%s mux=%s"
                    %
                    (
                        name,
                        str(self.point_status),
                        str(self.mux_status)
                    )
                )

            rospy.sleep(0.02)

        deadline = (
            time.time()
            +
            self.waypoint_timeout
        )

        stable_since = None
        last_print = 0.0

        while not rospy.is_shutdown():

            self.check_airborne()

            px, py, pz = self.position()

            dist = math.sqrt(
                (px - x) ** 2
                +
                (py - y) ** 2
                +
                (pz - z) ** 2
            )

            speed = self.speed()

            yaw_now = self.yaw()

            yaw_err_deg = math.degrees(
                wrap_pi(
                    yaw_now - yaw0
                )
            )

            if abs(yaw_err_deg) > self.yaw_abort_deg:

                self.fail(
                    "unexpected yaw deviation %.1f deg"
                    % yaw_err_deg
                )

            arrived = (
                self.point_status == "AIRBORNE_HOLD"
                and dist <= self.arrival_tol
                and speed <= self.speed_tol
            )

            now = time.time()

            if arrived:

                if stable_since is None:
                    stable_since = now

                if (
                    now - stable_since
                    >= self.arrival_stable_time
                ):

                    print(
                        GREEN +
                        "[ WAYPOINT REACHED ] %s "
                        "pos=(%.3f, %.3f, %.3f) "
                        "err=%.3f m yaw_err=%+.2f deg"
                        %
                        (
                            name,
                            px,
                            py,
                            pz,
                            dist,
                            yaw_err_deg
                        )
                        + RESET
                    )

                    break

            else:
                stable_since = None

            if now - last_print >= 0.5:

                print(
                    "%-8s "
                    "pos=(% .2f,% .2f,% .2f) "
                    "remain=%.2f "
                    "speed=%.2f "
                    "yaw_err=%+.1f "
                    "point=%s"
                    %
                    (
                        name,
                        px,
                        py,
                        pz,
                        dist,
                        speed,
                        yaw_err_deg,
                        str(self.point_status)
                    )
                )

                last_print = now

            if now > deadline:
                self.fail(
                    "%s waypoint timeout"
                    % name
                )

            rospy.sleep(0.05)

        # Explicit pause at each waypoint.
        hold_deadline = (
            time.time()
            +
            self.waypoint_hold_time
        )

        while (
            not rospy.is_shutdown()
            and time.time() < hold_deadline
        ):

            self.check_airborne()
            rospy.sleep(0.05)

    # ============================================================
    # Mission
    # ============================================================

    def run(self):

        self.wait_stack()
        self.wait_manual_arm()

        # --------------------------------------------
        # Reference defined BEFORE takeoff.
        # --------------------------------------------

        home_x, home_y, ground_z = self.position()
        yaw0 = self.yaw()

        rospy.set_param(
            "/uav2_touchdown/ground_z_reference",
            ground_z
        )

        print()
        print("========================================")
        print(" MISSION REFERENCE")
        print("========================================")
        print(
            "Home XY      : (%.3f, %.3f)"
            % (
                home_x,
                home_y
            )
        )
        print(
            "Ground Z     : %.3f"
            % ground_z
        )
        print(
            "Fixed yaw    : %.2f deg"
            % math.degrees(yaw0)
        )

        # --------------------------------------------
        # Frozen takeoff
        # --------------------------------------------

        self.run_flight_ops(
            "takeoff"
        )

        self.wait_takeoff_settle(
            ground_z
        )

        # POINT node keeps every horizontal goal at its
        # frozen HOME-relative takeoff altitude. Use exactly
        # the same reference here; never use a transient
        # measured z as the waypoint altitude.
        mission_z = (
            ground_z
            +
            self.takeoff_height
        )

        print(
            GREEN +
            "[ OK ] mission altitude reference = %.3f m"
            % mission_z
            + RESET
        )

        # --------------------------------------------
        # Four corners in INITIAL BODY frame.
        #
        # tuple = (name, forward, left)
        # --------------------------------------------

        route_body = [
            (
                "P1",
                -self.offset,
                +self.offset
            ),
            (
                "P2",
                +self.offset,
                +self.offset
            ),
            (
                "P3",
                +self.offset,
                -self.offset
            ),
            (
                "P4",
                -self.offset,
                -self.offset
            ),
            (
                "HOME",
                0.0,
                0.0
            )
        ]

        route_world = []

        for name, forward, left in route_body:

            wx, wy = self.body_to_world(
                home_x,
                home_y,
                yaw0,
                forward,
                left
            )

            route_world.append(
                (
                    name,
                    wx,
                    wy,
                    mission_z
                )
            )

        print()
        print("========================================")
        print(" FOUR-WAYPOINT ROUTE")
        print(" yaw remains fixed")
        print("========================================")

        for name, x, y, z in route_world:

            print(
                "%-5s : (% .3f, % .3f, %.3f)"
                % (
                    name,
                    x,
                    y,
                    z
                )
            )

        # --------------------------------------------
        # Execute sequential POINT trajectories
        # --------------------------------------------

        for name, x, y, z in route_world:

            self.fly_to(
                name,
                x,
                y,
                z,
                yaw0
            )

        print()
        print(
            GREEN +
            "========================================\n"
            " FOUR-WAYPOINT ROUTE COMPLETE\n"
            " Returned to takeoff XY\n"
            "========================================"
            + RESET
        )

        print()
        print("Hovering at HOME for 2.0 s ...")

        t_end = time.time() + 2.0

        while (
            not rospy.is_shutdown()
            and time.time() < t_end
        ):
            self.check_airborne()
            rospy.sleep(0.05)

        # --------------------------------------------
        # Frozen landing
        # --------------------------------------------

        print()
        print("========================================")
        print(" LANDING")
        print("========================================")

        self.run_flight_ops(
            "land"
        )

        print()
        print(
            GREEN +
            "========================================\n"
            " UAV2 FOUR-WAYPOINT DEMO COMPLETED\n"
            "========================================"
            + RESET
        )


if __name__ == "__main__":

    rospy.init_node(
        "uav2_square_demo"
    )

    node = UAV2SquareDemo()

    try:
        node.run()

    except RuntimeError as exc:
        rospy.logerr(str(exc))

    except rospy.ROSInterruptException:
        pass
