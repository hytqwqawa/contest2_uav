#!/usr/bin/env python3

import sys
import time
import rospy

from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


GREEN = "\033[1;32m"
RED = "\033[1;31m"
CYAN = "\033[1;36m"
YELLOW = "\033[1;33m"
RESET = "\033[0m"


class FlightOpsTest:

    def __init__(self):

        self.state = None
        self.odom = None

        self.health = None
        self.stream = None
        self.mission = None

        self.point_status = None
        self.mux_status = None

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

        self.services = {}

        names = [
            "request_offboard",
            "request_posctl",
            "disable_trajectory",
            "use_point",
            "abort_trajectory",
            "start_takeoff",
            "start_land",
        ]

        for name in names:

            full = "/ego_uav_interface/" + name

            rospy.wait_for_service(
                full,
                timeout=5.0
            )

            self.services[name] = rospy.ServiceProxy(
                full,
                Trigger
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
        self.point_status = msg.data


    def mux_cb(self, msg):
        self.mux_status = msg.data


    def z(self):

        if self.odom is None:
            return float("nan")

        return self.odom.pose.pose.position.z


    def wait_topics(self):

        deadline = time.time() + 4.0

        while not rospy.is_shutdown():

            if (
                self.state is not None
                and self.odom is not None
                and self.health is not None
                and self.stream is not None
                and self.mission is not None
                and self.point_status is not None
                and self.mux_status is not None
            ):
                return True

            if time.time() > deadline:
                return False

            rospy.sleep(0.02)

        return False


    def wait_mode(self, mode, timeout):

        deadline = time.time() + timeout

        while not rospy.is_shutdown():

            if (
                self.state is not None
                and self.state.mode == mode
            ):
                return True

            if time.time() > deadline:
                return False

            rospy.sleep(0.02)

        return False


    def wait_mux_point(self, timeout=1.0):

        deadline = time.time() + timeout

        while not rospy.is_shutdown():

            if self.mux_status == "POINT_ACTIVE":
                return True

            if time.time() > deadline:
                return False

            rospy.sleep(0.02)

        return False


    def call(self, name, require=True):

        try:

            r = self.services[name]()

        except rospy.ServiceException as exc:

            print(
                RED +
                "[SERVICE ERROR] %s: %s"
                % (name, str(exc))
                + RESET
            )

            return None

        print(
            "%-20s success=%s  %s"
            %
            (
                name,
                r.success,
                r.message
            )
        )

        if require and not r.success:
            return None

        return r


    def show_initial(self):

        print()
        print("========================================")
        print(" FLIGHT OPS TEST")
        print("========================================")

        print("armed          :", self.state.armed)
        print("mode           :", self.state.mode)
        print("health         :", self.health)
        print("stream         :", self.stream)
        print("mission        :", self.mission)
        print("point          :", self.point_status)
        print("mux            :", self.mux_status)
        print("z              : %.3f" % self.z())
        print()


    def prepare_point_control(self):

        print(
            CYAN +
            "[1] Preparing POINT control source..."
            + RESET
        )

        # Stop whatever trajectory is currently reaching PX4.
        r = self.call(
            "disable_trajectory",
            require=False
        )

        rospy.sleep(0.10)

        # Capture the CURRENT aircraft position into point node.
        # This is critical after an EGO mission.
        r = self.call(
            "abort_trajectory",
            require=True
        )

        if r is None:
            return False

        rospy.sleep(0.10)

        r = self.call(
            "use_point",
            require=True
        )

        if r is None:
            return False

        if not self.wait_mux_point(1.0):

            print(
                RED +
                "[ABORT] mux did not become POINT_ACTIVE"
                + RESET
            )

            return False

        print(
            GREEN +
            "[ OK ] mux = POINT_ACTIVE"
            + RESET
        )

        return True


    def ensure_offboard(self):

        if self.state.mode == "OFFBOARD":
            return True

        if self.state.mode != "POSCTL":

            print(
                RED +
                "[ABORT] PX4 must be POSCTL or OFFBOARD. mode=%s"
                % self.state.mode
                + RESET
            )

            return False

        print(
            CYAN +
            "[2] Requesting OFFBOARD..."
            + RESET
        )

        r = self.call(
            "request_offboard",
            require=True
        )

        if r is None:
            return False

        if not self.wait_mode(
            "OFFBOARD",
            1.5
        ):

            print(
                RED +
                "[ABORT] PX4 did not enter OFFBOARD"
                + RESET
            )

            return False

        # Give point_traj/offboard_manager callbacks
        # a short time to observe the confirmed mode.
        rospy.sleep(0.25)

        print(
            GREEN +
            "[ OK ] PX4 = OFFBOARD"
            + RESET
        )

        return True


    def takeoff(self):

        self.show_initial()

        if not self.state.armed:

            print(
                RED +
                "[ABORT] ARM with physical CH5 first."
                + RESET
            )

            return 1

        if not self.health:

            print(
                RED +
                "[ABORT] flight_healthy is FALSE"
                + RESET
            )

            return 1

        if not self.stream:

            print(
                RED +
                "[ABORT] OFFBOARD stream not ready"
                + RESET
            )

            return 1

        if not self.mission:

            print(
                RED +
                "[ABORT] CH7 mission permission is FALSE"
                + RESET
            )

            return 1

        if self.point_status not in [
            "HOLD_GROUND",
            "LANDED_HOLD"
        ]:

            print(
                RED +
                "[ABORT] point state is not ground HOLD: %s"
                % self.point_status
                + RESET
            )

            return 1

        if not self.prepare_point_control():
            return 1

        if not self.ensure_offboard():
            return 1

        print(
            CYAN +
            "[3] Starting TAKEOFF..."
            + RESET
        )

        # Retry ONLY the known OFFBOARD callback synchronization race.
        accepted = False

        for attempt in range(1, 4):

            r = self.call(
                "start_takeoff",
                require=False
            )

            if r is not None and r.success:

                accepted = True
                break

            msg = (
                r.message
                if r is not None
                else ""
            )

            if (
                "ARMED and OFFBOARD" not in msg
                and
                "connected, ARMED and OFFBOARD" not in msg
            ):

                print(
                    RED +
                    "[ABORT] takeoff rejected for a real reason"
                    + RESET
                )

                return 1

            print(
                YELLOW +
                "[ retry %d/3 ] waiting for OFFBOARD callback sync..."
                % attempt
                + RESET
            )

            rospy.sleep(0.25)

        if not accepted:

            print(
                RED +
                "[ABORT] TAKEOFF not accepted after retries"
                + RESET
            )

            return 1

        z0 = self.z()

        print(
            GREEN +
            "[ ACCEPTED ] TAKEOFF z0=%.3f"
            % z0
            + RESET
        )

        deadline = time.time() + 9.0
        last_print = 0.0

        while not rospy.is_shutdown():

            now = time.time()

            if (
                now - last_print
                >= 0.10
            ):

                print(
                    "\r\033[2K"
                    "TAKEOFF  z=%6.3f  dz=%5.3f  "
                    "mode=%-8s  point=%s"
                    %
                    (
                        self.z(),
                        self.z() - z0,
                        self.state.mode,
                        self.point_status
                    ),
                    end="",
                    flush=True
                )

                last_print = now

            if not self.state.armed:

                print()
                print(
                    RED +
                    "[ABORT] vehicle disarmed during takeoff"
                    + RESET
                )
                return 1

            if self.state.mode != "OFFBOARD":

                print()
                print(
                    RED +
                    "[ABORT] PX4 left OFFBOARD -> %s"
                    % self.state.mode
                    + RESET
                )
                return 1

            if self.point_status == "AIRBORNE_HOLD":

                print()

                print(
                    GREEN +
                    "========================================"
                    + RESET
                )

                print(
                    GREEN +
                    " TAKEOFF COMPLETE  z=%.3f"
                    % self.z()
                    + RESET
                )

                print(
                    GREEN +
                    "========================================"
                    + RESET
                )

                return 0

            if now > deadline:

                print()

                print(
                    RED +
                    "[ABORT] takeoff timeout, point=%s z=%.3f"
                    %
                    (
                        self.point_status,
                        self.z()
                    )
                    + RESET
                )

                return 1

            rospy.sleep(0.02)

        return 1


    def land(self):

        self.show_initial()

        if not self.state.armed:

            print(
                RED +
                "[ABORT] vehicle is not armed"
                + RESET
            )
            return 1

        if not self.health or not self.mission:

            print(
                RED +
                "[ABORT] health/mission permission invalid"
                + RESET
            )
            return 1

        if self.state.mode != "OFFBOARD":

            print(
                RED +
                "[ABORT] landing test starts from OFFBOARD. mode=%s"
                % self.state.mode
                + RESET
            )
            return 1

        z0 = self.z()

        if not self.prepare_point_control():
            return 1

        # Still require OFFBOARD after source handoff.
        if self.state.mode != "OFFBOARD":

            print(
                RED +
                "[ABORT] lost OFFBOARD during point handoff"
                + RESET
            )
            return 1

        rospy.sleep(0.20)

        print(
            CYAN +
            "[2] Starting LAND..."
            + RESET
        )

        r = self.call(
            "start_land",
            require=True
        )

        if r is None:
            return 1

        print(
            GREEN +
            "[ ACCEPTED ] LAND z0=%.3f"
            % z0
            + RESET
        )

        deadline = time.time() + 9.0
        last_print = 0.0

        while not rospy.is_shutdown():

            now = time.time()

            if (
                now - last_print
                >= 0.10
            ):

                print(
                    "\r\033[2K"
                    "LAND     z=%6.3f  drop=%5.3f  "
                    "mode=%-8s  point=%s"
                    %
                    (
                        self.z(),
                        z0 - self.z(),
                        self.state.mode,
                        self.point_status
                    ),
                    end="",
                    flush=True
                )

                last_print = now

            if not self.state.armed:

                print()

                print(
                    GREEN +
                    "Vehicle auto-disarmed during landing."
                    + RESET
                )

                return 0

            if self.state.mode != "OFFBOARD":

                print()

                print(
                    RED +
                    "[ABORT] PX4 left OFFBOARD unexpectedly -> %s"
                    % self.state.mode
                    + RESET
                )

                return 1

            if self.point_status == "LANDED_HOLD":

                print()

                print(
                    GREEN +
                    "[ OK ] LAND trajectory complete, z=%.3f"
                    % self.z()
                    + RESET
                )

                break

            if now > deadline:

                print()

                print(
                    RED +
                    "[ABORT] landing trajectory timeout, z=%.3f"
                    % self.z()
                    + RESET
                )

                return 1

            rospy.sleep(0.02)

        # Stop P/V/A trajectory commands before returning to POSCTL.
        self.call(
            "disable_trajectory",
            require=False
        )

        rospy.sleep(0.10)

        print(
            CYAN +
            "[3] Requesting POSCTL for final ground/disarm phase..."
            + RESET
        )

        self.call(
            "request_posctl",
            require=False
        )

        deadline = time.time() + 4.0

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
        ):

            print(
                "\r\033[2K"
                "GROUND   z=%6.3f  armed=%s  mode=%s"
                %
                (
                    self.z(),
                    self.state.armed,
                    self.state.mode
                ),
                end="",
                flush=True
            )

            if not self.state.armed:

                print()

                print(
                    GREEN +
                    "========================================"
                    + RESET
                )

                print(
                    GREEN +
                    " LANDING COMPLETE - DISARMED"
                    + RESET
                )

                print(
                    GREEN +
                    "========================================"
                    + RESET
                )

                return 0

            rospy.sleep(0.10)

        print()

        print(
            YELLOW +
            "[WARN] Aircraft reached landing state but remains armed."
            + RESET
        )

        print(
            YELLOW +
            "If physically on the ground, disarm with CH5."
            + RESET
        )

        return 0


def main():

    if len(sys.argv) < 2:

        print(
            "Usage: flight_ops_test.py takeoff|land"
        )

        return 1

    command = sys.argv[1].lower()

    rospy.init_node(
        "flight_ops_test",
        anonymous=True
    )

    node = FlightOpsTest()

    if not node.wait_topics():

        print(
            RED +
            "[ABORT] required topics did not become ready"
            + RESET
        )

        return 1

    if command == "takeoff":
        return node.takeoff()

    if command == "land":
        return node.land()

    print(
        "Unknown command: %s"
        % command
    )

    return 1


if __name__ == "__main__":
    sys.exit(main())
