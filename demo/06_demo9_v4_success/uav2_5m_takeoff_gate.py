#!/usr/bin/env python3

import subprocess
import time
from pathlib import Path

import rospy

from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class Demo4TakeoffGate:

    def __init__(self):

        self.state = None
        self.odom = None

        self.health = False
        self.health_seen = False

        self.stream = False
        self.stream_seen = False

        self.mission = False
        self.mission_seen = False

        self.point_status = ""
        self.manager_status = ""

        rospy.Subscriber(
            "/contest2/mavros/state",
            State,
            self.state_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/mavros/local_position/odom",
            Odometry,
            self.odom_cb,
            queue_size=50
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/flight_healthy",
            Bool,
            self.health_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/offboard_stream_ready",
            Bool,
            self.stream_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/rc_mission_allowed",
            Bool,
            self.mission_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/point_traj_status",
            String,
            self.point_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/offboard_manager_status",
            String,
            self.manager_cb,
            queue_size=20
        )

    def state_cb(self, msg):
        self.state = msg

    def odom_cb(self, msg):
        self.odom = msg

    def health_cb(self, msg):
        self.health = bool(msg.data)
        self.health_seen = True

    def stream_cb(self, msg):
        self.stream = bool(msg.data)
        self.stream_seen = True

    def mission_cb(self, msg):
        self.mission = bool(msg.data)
        self.mission_seen = True

    def point_cb(self, msg):
        self.point_status = msg.data

    def manager_cb(self, msg):
        self.manager_status = msg.data

    def fail(self, reason):

        print()
        print("[ABORT] " + reason)

        if (
            self.state is not None
            and self.state.armed
        ):
            print()
            print("========================================")
            print(" PILOT TAKEOVER REQUIRED")
            print(" CH7 -> LOW")
            print(" CH6 -> POSCTL")
            print(" Keep CH5 ARM until aircraft is safe.")
            print("========================================")
            print()

            while (
                not rospy.is_shutdown()
                and self.state is not None
                and self.state.armed
                and self.state.mode == "OFFBOARD"
            ):
                rospy.sleep(0.10)

        return 1

    def wait_inputs(self):

        deadline = time.time() + 12.0

        while not rospy.is_shutdown():

            ready = (
                self.state is not None
                and self.odom is not None
                and self.health_seen
                and self.stream_seen
                and self.mission_seen
                and self.point_status != ""
            )

            if ready:
                return True

            if time.time() > deadline:
                return False

            rospy.sleep(0.05)

        return False

    def wait_pilot(self):

        print()
        print("========================================")
        print(" DEMO05 WAYPOINT READY")
        print(" Demo3 TAKEOFF -> MULTI-WAYPOINT EGO -> Demo3 LAND")
        print("========================================")
        print()
        print("NO SOFTWARE ARM / DISARM")
        print()
        print("WAITING FOR PILOT")
        print(" CH7 : LOW -> MID -> HIGH")
        print(" CH6 : POSCTL")
        print(" CH5 : ARM")
        print()
        print(
            "After ARM, keep CH7 HIGH and CH6 POSCTL "
            "until the script changes mode itself."
        )
        print()

        last_print = 0.0

        while not rospy.is_shutdown():

            if (
                self.state is None
                or not self.state.connected
            ):
                return self.fail("PX4 link lost")

            # PROVEN_DEMO3_PILOT_GATE
            #
            # Match the successfully flown Demo3:
            # physical ARM + CH7 permission + valid pilot mode.
            #
            # Do NOT duplicate offboard_manager qualification here.
            # uav2_flight_ops_dev.py performs the authoritative
            # health / stream / POINT / OFFBOARD checks immediately
            # before takeoff.
            if (
                self.state.armed
                and self.mission
                and self.state.mode
                in ["POSCTL", "OFFBOARD"]
            ):
                print()
                print("[ OK ] PILOT AUTHORIZATION RECEIVED")
                return 0

            now = time.time()

            if now - last_print >= 0.50:

                print(
                    "\rWAIT "
                    "armed=%s mode=%-8s CH7=%s "
                    "health=%s stream=%s point=%s"
                    %
                    (
                        self.state.armed,
                        self.state.mode,
                        self.mission,
                        self.health,
                        self.stream,
                        self.point_status,
                    ),
                    end="",
                    flush=True
                )

                last_print = now

            rospy.sleep(0.05)

        return 1


    def wait_takeoff_settle(self, ground_z):

        target_z = ground_z + 0.80

        stable_since = None
        deadline = time.time() + 15.0
        last_print = 0.0

        while not rospy.is_shutdown():

            if not self.health:
                return self.fail(
                    "flight_healthy lost during takeoff settle"
                )

            if not self.mission:
                return self.fail(
                    "CH7 mission permission lost during takeoff settle"
                )

            if (
                self.state is None
                or not self.state.armed
                or self.state.mode != "OFFBOARD"
            ):
                return self.fail(
                    "PX4 left normal OFFBOARD state during takeoff settle"
                )

            if self.odom is None:
                rospy.sleep(0.02)
                continue

            z = self.odom.pose.pose.position.z
            vz = self.odom.twist.twist.linear.z
            err = z - target_z

            good = (
                self.point_status == "AIRBORNE_HOLD"
                and abs(err) <= 0.06
                and abs(vz) <= 0.10
            )

            now = time.time()

            if now - last_print >= 0.20:

                print(
                    "\rTAKEOFF SETTLE "
                    "z=%+.3f target=%+.3f "
                    "err=%+.3f vz=%+.3f point=%s"
                    %
                    (
                        z,
                        target_z,
                        err,
                        vz,
                        self.point_status
                    ),
                    end="",
                    flush=True
                )

                last_print = now

            if good:

                if stable_since is None:
                    stable_since = now

                elif now - stable_since >= 0.8:
                    print()
                    print("[ OK ] TAKEOFF PHYSICALLY SETTLED")
                    return 0

            else:
                stable_since = None

            if now > deadline:
                print()
                return self.fail("takeoff settle timeout")

            rospy.sleep(0.05)

        return 1

    def run(self):

        if not self.wait_inputs():
            return self.fail(
                "required core / POINT topics did not become ready"
            )

        if self.state.armed:
            return self.fail(
                "vehicle must be DISARMED when Terminal 3 starts"
            )

        if self.state.mode != "POSCTL":
            return self.fail(
                "vehicle must be POSCTL before pilot authorization"
            )

        if self.mission:
            return self.fail(
                "CH7 must be LOW when Terminal 3 starts"
            )

        if not self.health:
            return self.fail(
                "flight_healthy is FALSE before ARM"
            )

        if not self.stream:
            return self.fail(
                "OFFBOARD stream is not ready before ARM"
            )

        if self.point_status not in [
            "HOLD_GROUND",
            "LANDED_HOLD"
        ]:
            return self.fail(
                "POINT state is not ground HOLD: %s"
                % self.point_status
            )

        ground_z = self.odom.pose.pose.position.z

        rospy.set_param(
            "/contest2/uav2_touchdown/ground_z_reference",
            float(ground_z)
        )

        print(
            "[ HOME ] ground_z_reference = %.3f"
            % ground_z
        )

        ret = self.wait_pilot()

        if ret != 0:
            return ret

        script = (
            Path(__file__).resolve().parent
            / "uav2_flight_ops_dev.py"
        )

        if not script.is_file():
            return self.fail(
                "Demo3 local flight_ops missing: %s"
                % script
            )

        print()
        print("========================================")
        print(" DEMO3 TAKEOFF")
        print("========================================")
        print()

        ret = subprocess.call(
            [
                "python3",
                str(script),
                "takeoff"
            ]
        )

        if ret != 0:
            return self.fail(
                "Demo3 local takeoff failed "
                "with return code %d" % ret
            )

        return self.wait_takeoff_settle(
            ground_z
        )


def main():

    rospy.init_node(
        "demo04_5m_takeoff_gate",
        anonymous=False
    )

    gate = Demo4TakeoffGate()

    try:
        return gate.run()

    except rospy.ROSInterruptException:
        return 1

    except Exception as exc:
        return gate.fail(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())
