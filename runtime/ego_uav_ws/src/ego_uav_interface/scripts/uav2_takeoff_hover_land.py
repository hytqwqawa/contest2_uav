#!/usr/bin/env python3

import os
import subprocess
import time
import rospy

from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


GREEN = "\033[1;32m"
RED = "\033[1;31m"
CYAN = "\033[1;36m"
RESET = "\033[0m"


class UAV2TakeoffHoverLand:

    def __init__(self):

        self.state = None
        self.odom = None

        self.health = False
        self.stream = False
        self.mission = False

        self.point = None
        self.mux = None

        self.hover_time = rospy.get_param(
            "~hover_time",
            5.0
        )

        # Same physical-settle idea as UAV1.
        self.min_climb = rospy.get_param(
            "~min_climb",
            0.60
        )

        self.vz_limit = rospy.get_param(
            "~vz_limit",
            0.10
        )

        self.settle_time = rospy.get_param(
            "~settle_time",
            0.60
        )

        self.settle_timeout = rospy.get_param(
            "~settle_timeout",
            8.0
        )

        self.flight_ops_script = os.path.join(
            os.path.dirname(
                os.path.abspath(__file__)
            ),
            "uav2_flight_ops_test.py"
        )

        rospy.Subscriber(
            "/uav1/mavros/state",
            State,
            self.state_cb,
            queue_size=10
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
            "/ego_uav_interface/point_traj_status",
            String,
            self.point_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/ego_uav_interface/trajectory_mux_status",
            String,
            self.mux_cb,
            queue_size=10
        )


    def state_cb(self, msg):
        self.state = msg


    def odom_cb(self, msg):
        self.odom = msg


    def health_cb(self, msg):
        self.health = msg.data


    def stream_cb(self, msg):
        self.stream = msg.data


    def mission_cb(self, msg):
        self.mission = msg.data


    def point_cb(self, msg):
        self.point = msg.data


    def mux_cb(self, msg):
        self.mux = msg.data


    def fail(self, text):
        raise RuntimeError(text)


    def wait_stack(self):

        print()
        print("========================================")
        print(" UAV2 TAKEOFF-HOVER-LAND INITIALIZING")
        print("========================================")

        deadline = time.time() + 20.0

        while not rospy.is_shutdown():

            ready = (
                self.state is not None
                and self.state.connected
                and self.odom is not None
                and self.health
                and self.stream
                and self.point is not None
                and self.mux is not None
            )

            if ready:
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

        print()
        print(
            "ROUTE: TAKEOFF 0.80 m "
            "-> HOVER %.1f s -> LAND"
            % self.hover_time
        )

        print("NO AUTOMATIC ARM")


    def wait_manual_arm(self):

        # Exactly the UAV1 operator logic.
        print()
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
                and self.mission
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


    def run_flight_ops(self, command):

        print()
        print(
            CYAN +
            "[ FLIGHT OPS ] %s"
            % command.upper()
            + RESET
        )

        ret = subprocess.call([
            self.flight_ops_script,
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

        deadline = (
            time.time()
            +
            self.settle_timeout
        )

        stable_since = None
        last_print = 0.0

        print()
        print(
            CYAN +
            "[ TAKEOFF SETTLE ] "
            "waiting for physical altitude..."
            + RESET
        )

        while not rospy.is_shutdown():

            if self.odom is None:
                rospy.sleep(0.02)
                continue

            z = (
                self.odom.pose.pose.position.z
            )

            vz = (
                self.odom.twist.twist.linear.z
            )

            dz = z - ground_z

            now = time.time()

            if (
                now - last_print
                >= 0.15
            ):

                print(
                    "\r\033[2K"
                    "SETTLE "
                    "z=%6.3f "
                    "dz=%5.3f "
                    "vz=%+5.2f m/s"
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
                dz >= self.min_climb
                and
                abs(vz) <= self.vz_limit
            )

            if good:

                if stable_since is None:
                    stable_since = now

                elif (
                    now - stable_since
                    >= self.settle_time
                ):

                    print()

                    print(
                        GREEN +
                        "[ OK ] TAKEOFF "
                        "PHYSICALLY SETTLED "
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
                    "vehicle disarmed during "
                    "takeoff settle"
                )

            if (
                self.state.mode
                != "OFFBOARD"
            ):

                print()

                self.fail(
                    "PX4 left OFFBOARD during "
                    "takeoff settle"
                )

            if now > deadline:

                print()

                self.fail(
                    "takeoff did not settle: "
                    "z=%.3f dz=%.3f vz=%.3f"
                    %
                    (
                        z,
                        dz,
                        vz
                    )
                )

            rospy.sleep(0.02)


    def hover(self):

        print()
        print(
            CYAN +
            "Hovering for %.1f s ..."
            %
            self.hover_time
            + RESET
        )

        end = (
            time.time()
            +
            self.hover_time
        )

        while (
            time.time() < end
            and not rospy.is_shutdown()
        ):

            if (
                self.state is None
                or not self.state.armed
                or self.state.mode
                    != "OFFBOARD"
                or not self.health
            ):

                self.fail(
                    "flight state lost during hover"
                )

            rospy.sleep(0.1)


    def run(self):

        self.wait_stack()

        self.wait_manual_arm()

        if self.odom is None:
            self.fail(
                "no odometry before takeoff"
            )

        ground_z = (
            self.odom.pose.pose.position.z
        )

        # Store the actual pre-takeoff ground reference.
        # The landing subprocess uses the same PX4 local-Z frame.
        rospy.set_param(
            "/uav2_touchdown/ground_z_reference",
            ground_z
        )

        print(
            "[ GROUND REF ] z=%.3f m"
            % ground_z
        )

        print()
        print("========================================")
        print(" TAKEOFF")
        print("========================================")

        self.run_flight_ops(
            "takeoff"
        )

        self.wait_takeoff_settle(
            ground_z
        )

        self.hover()

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
            "========================================"
            + RESET
        )

        print(
            GREEN +
            " UAV2 TAKEOFF-HOVER-LAND COMPLETED"
            + RESET
        )

        print(
            GREEN +
            "========================================"
            + RESET
        )


if __name__ == "__main__":

    rospy.init_node(
        "uav2_takeoff_hover_land"
    )

    node = UAV2TakeoffHoverLand()

    try:

        node.run()

    except Exception as exc:

        print()

        print(
            RED +
            "MISSION STOPPED: %s"
            % str(exc)
            + RESET,
            flush=True
        )

        print(
            "CH6 remains the hardware/manual "
            "takeover path.",
            flush=True
        )

        raise
