#!/usr/bin/env python3

import math
import os
import subprocess
import threading
import time
from pathlib import Path

import rospy
import yaml

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float64
from std_srvs.srv import Trigger


class WaypointPoleMission:

    def __init__(self):

        self.root = Path(__file__).resolve().parent

        self.mission_file = (
            self.root / "mission.yaml"
        )

        self.session_file = (
            self.root / "session_frame.yaml"
        )

        self.odom = None
        self.state = None

        self.health = False
        self.stream = False
        self.mission_allowed = False
        self.demo_safe = False

        self.point_status = ""
        self.mux_status = ""
        self.goal_status = ""

        self.goal_done = threading.Event()
        self.goal_idle = threading.Event()
        self.goal_abort = None

        rospy.Subscriber(
            "/contest2/mavros/local_position/odom",
            Odometry,
            self.odom_cb,
            queue_size=30
        )

        rospy.Subscriber(
            "/contest2/mavros/state",
            State,
            self.state_cb,
            queue_size=20
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
            "/contest2/ego_uav_interface/demo_frame_safe",
            Bool,
            self.safe_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/point_traj_status",
            String,
            self.point_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/trajectory_mux_status",
            String,
            self.mux_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/rviz_goal_status",
            String,
            self.goal_status_cb,
            queue_size=50
        )

        self.goal_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/rviz_goal",
            PoseStamped,
            queue_size=1
        )


        # P1 -> P2 obstacle pre-look:
        # hold the final P1 position while rotating the camera
        # toward the next leg before EGO plans around the pole.
        self.yaw_mode_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/yaw_mode",
            String,
            queue_size=1,
            latch=True
        )

        self.yaw_target_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/yaw_target_deg",
            Float64,
            queue_size=1,
            latch=True
        )

        self.load_config()


    # ========================================================
    # callbacks
    # ========================================================

    def odom_cb(self, msg):
        self.odom = msg


    def state_cb(self, msg):
        self.state = msg


    def health_cb(self, msg):
        self.health = msg.data


    def stream_cb(self, msg):
        self.stream = msg.data


    def mission_cb(self, msg):
        self.mission_allowed = msg.data


    def safe_cb(self, msg):
        self.demo_safe = msg.data


    def point_cb(self, msg):
        self.point_status = msg.data


    def mux_cb(self, msg):
        self.mux_status = msg.data


    def goal_status_cb(self, msg):

        self.goal_status = msg.data

        if msg.data.startswith("ABORT"):
            self.goal_abort = msg.data

        if msg.data.startswith("DONE"):
            self.goal_done.set()

        if msg.data == "IDLE":
            self.goal_idle.set()


    # ========================================================
    # configuration
    # ========================================================

    def load_config(self):

        with open(
            self.mission_file,
            "r"
        ) as f:
            self.cfg = yaml.safe_load(f)

        with open(
            self.session_file,
            "r"
        ) as f:
            self.session = yaml.safe_load(f)

        o = self.session["origin_world"]
        fw = self.session["forward_world"]
        lw = self.session["left_world"]

        self.ox = float(o["x"])
        self.oy = float(o["y"])
        self.oz = float(o["z"])

        self.fx = float(fw["x"])
        self.fy = float(fw["y"])

        self.lx = float(lw["x"])
        self.ly = float(lw["y"])

        self.height = float(
            self.cfg["flight_height"]
        )

        self.waypoints = (
            self.cfg["waypoints"]
        )


    def demo_to_world(
        self,
        f,
        l,
        h
    ):

        return (
            self.ox
            + self.fx * f
            + self.lx * l,

            self.oy
            + self.fy * f
            + self.ly * l,

            self.oz + h
        )


    # ========================================================
    # safety
    # ========================================================

    def safe_hold(self):

        for name in [
            "/contest2/ego_uav_interface/disable_trajectory",
            "/contest2/ego_uav_interface/abort_trajectory",
            "/contest2/ego_uav_interface/use_point",
        ]:

            try:
                rospy.wait_for_service(
                    name,
                    timeout=0.5
                )

                rospy.ServiceProxy(
                    name,
                    Trigger
                )()

            except Exception:
                pass


    def fail(self, reason):

        rospy.logerr(
            "WAYPOINT DEMO ABORT: %s",
            reason
        )

        self.safe_hold()

        raise RuntimeError(reason)


    def check_runtime(self):

        if not self.health:
            self.fail(
                "flight_healthy=False"
            )

        if not self.mission_allowed:
            self.fail(
                "CH7 mission permission lost"
            )

        if not self.demo_safe:
            self.fail(
                "Demo-frame guard UNSAFE"
            )

        if (
            self.state is None
            or not self.state.armed
        ):
            self.fail(
                "vehicle disarmed"
            )

        if (
            self.state.mode
            != "OFFBOARD"
        ):
            self.fail(
                "PX4 left OFFBOARD"
            )


    # ========================================================
    # flight ops
    # ========================================================

    def run_flight_ops(self, command):

        script = str(
            Path.home()
            / "contest2_uav"
            / "runtime"
            / "ego_uav_ws"
            / "src"
            / "ego_uav_interface"
            / "scripts"
            / "uav2_flight_ops_test.py"
        )

        print()
        print(
            "[ FLIGHT OPS ] %s"
            % command.upper()
        )

        ret = subprocess.call([
            "python3",
            script,
            command
        ])

        if ret != 0:
            self.fail(
                "flight_ops %s failed"
                % command
            )


    # ========================================================
    # waits
    # ========================================================

    def wait_stack(self):

        print()
        print("========================================")
        print(" UAV2 WAYPOINT-POLE DEMO")
        print("========================================")
        print()
        print("Route:")
        print(" P0 -> P1 -> P2 -> P3 -> LAND")
        print()
        print("Pole:")
        print(" F=+1.0  L=+2.0")
        print()
        print("NO SOFTWARE ARM")
        print()

        deadline = time.time() + 20.0

        while not rospy.is_shutdown():

            ready = (
                self.odom is not None
                and
                self.state is not None
                and
                self.state.connected
                and
                self.health
                and
                self.stream
                and
                self.goal_pub.get_num_connections() > 0
            )

            if ready:
                return

            if time.time() > deadline:
                self.fail(
                    "stack not ready"
                )

            rospy.sleep(0.1)


    def wait_manual_arm(self):

        print()
        print("----------------------------------------")
        print(" WAITING FOR PILOT")
        print(" CH7 : LOW -> MID -> HIGH")
        print(" CH6 : POSCTL")
        print(" CH5 : ARM")
        print("----------------------------------------")
        print()

        while not rospy.is_shutdown():

            if (
                self.state is not None
                and
                self.state.armed
                and
                self.mission_allowed
                and
                self.state.mode
                in ["POSCTL", "OFFBOARD"]
            ):
                break

            rospy.sleep(0.10)

        print(
            "[ OK ] PILOT AUTHORIZATION RECEIVED"
        )


    def wait_takeoff_settle(self):

        target_z = (
            self.oz + self.height
        )

        stable_since = None
        deadline = time.time() + 15.0

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            self.check_runtime()

            p = self.odom.pose.pose.position
            vz = self.odom.twist.twist.linear.z

            ez = p.z - target_z

            good = (
                abs(ez) <= 0.06
                and
                abs(vz) <= 0.10
            )

            print(
                "\rTAKEOFF SETTLE "
                "z=%+.3f target=%+.3f "
                "err=%+.3f vz=%+.3f"
                %
                (
                    p.z,
                    target_z,
                    ez,
                    vz
                ),
                end="",
                flush=True
            )

            if good:

                if stable_since is None:
                    stable_since = time.time()

                elif (
                    time.time()
                    - stable_since
                    >= 0.8
                ):
                    print()
                    print(
                        "[ OK ] TAKEOFF PHYSICALLY SETTLED"
                    )
                    return

            else:
                stable_since = None

            if time.time() > deadline:
                print()
                self.fail(
                    "takeoff settle timeout"
                )

            rate.sleep()


    # ========================================================
    # obstacle pre-look
    # ========================================================

    @staticmethod
    def yaw_from_quaternion(q):

        return math.atan2(
            2.0 * (
                q.w * q.z
                +
                q.x * q.y
            ),
            1.0
            -
            2.0 * (
                q.y * q.y
                +
                q.z * q.z
            )
        )


    def prelook_to_waypoint(
        self,
        wp
    ):

        self.check_runtime()

        f = float(wp["forward"])
        l = float(wp["left"])

        gx, gy, _ = self.demo_to_world(
            f,
            l,
            self.height
        )

        p = self.odom.pose.pose.position

        target_yaw = math.atan2(
            gy - p.y,
            gx - p.x
        )

        target_deg = math.degrees(
            target_yaw
        )

        print()
        print("========================================")
        print(" P1 -> P2 OBSTACLE PRE-LOOK")
        print("========================================")
        print(
            "rotate camera toward P2: %.1f deg"
            % target_deg
        )

        self.yaw_target_pub.publish(
            Float64(data=target_deg)
        )

        self.yaw_mode_pub.publish(
            String(data="FIXED")
        )

        stable_since = None
        deadline = time.time() + 8.0

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            self.check_runtime()

            q = self.odom.pose.pose.orientation

            yaw = self.yaw_from_quaternion(q)

            err = math.atan2(
                math.sin(
                    target_yaw - yaw
                ),
                math.cos(
                    target_yaw - yaw
                )
            )

            good = (
                abs(err)
                <= math.radians(8.0)
            )

            print(
                "\rPRELOOK yaw=%+7.1f "
                "target=%+7.1f "
                "err=%+6.1f deg"
                %
                (
                    math.degrees(yaw),
                    target_deg,
                    math.degrees(err)
                ),
                end="",
                flush=True
            )

            if good:

                if stable_since is None:
                    stable_since = time.time()

                elif (
                    time.time()
                    -
                    stable_since
                    >= 0.50
                ):
                    print()
                    break

            else:
                stable_since = None

            if time.time() > deadline:
                print()
                self.fail(
                    "P2 obstacle pre-look timeout"
                )

            rate.sleep()

        print(
            "[ MAP SETTLE ] camera faces P2; "
            "waiting 1.0 s for D435/EGO map"
        )

        end = time.time() + 1.0

        while time.time() < end:

            self.check_runtime()
            rospy.sleep(0.05)

        print(
            "[ OK ] P2 obstacle pre-look complete"
        )


    # ========================================================
    # one EGO leg
    # ========================================================

    def execute_leg(
        self,
        wp,
        settle_after=0.8
    ):

        name = str(wp["name"])

        f = float(wp["forward"])
        l = float(wp["left"])

        gx, gy, gz = self.demo_to_world(
            f,
            l,
            self.height
        )

        # WAYPOINT_POLE_P3_LANDING_RADIUS_V1
        #
        # Keep P1/P2 precise.
        # P3 is immediately followed by LAND and therefore
        # uses a 0.25-m terminal compatibility radius.
        if name == "P3":
            arrive_xy = 0.25
        else:
            arrive_xy = 0.15

        rospy.set_param(
            "/contest2/waypoint_pole/arrive_xy",
            arrive_xy
        )

        print(
            "[ ARRIVAL ] %s dXY < %.2f m"
            % (name, arrive_xy)
        )

        self.goal_done.clear()
        self.goal_idle.clear()
        self.goal_abort = None

        print()
        print("========================================")
        print(" %s" % name)
        print("========================================")
        print(
            "DEMO  F=%+.2f L=%+.2f H=%.2f"
            %
            (
                f,
                l,
                self.height
            )
        )

        print(
            "WORLD x=%+.3f y=%+.3f z=%+.3f"
            %
            (
                gx,
                gy,
                gz
            )
        )

        msg = PoseStamped()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"

        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.position.z = gz
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)

        start = time.time()
        deadline = start + 70.0

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            self.check_runtime()

            if self.goal_abort is not None:
                self.fail(
                    "%s: %s"
                    %
                    (
                        name,
                        self.goal_abort
                    )
                )

            if self.goal_done.is_set():
                break

            if time.time() > deadline:
                self.fail(
                    "%s timeout"
                    % name
                )

            rate.sleep()

        # Executor publishes IDLE after DONE.
        idle_deadline = time.time() + 3.0

        while (
            not self.goal_idle.is_set()
            and
            time.time() < idle_deadline
        ):
            rospy.sleep(0.02)

        if not self.goal_idle.is_set():
            self.fail(
                "%s executor did not return IDLE"
                % name
            )

        p = self.odom.pose.pose.position

        print()
        print(
            "[ OK ] %s REACHED "
            "actual=(%.3f %.3f %.3f)"
            %
            (
                name,
                p.x,
                p.y,
                p.z
            )
        )

        print(
            "[ HOLD ] %.1f s"
            % settle_after
        )

        hold_end = (
            time.time()
            +
            settle_after
        )

        while (
            not rospy.is_shutdown()
            and
            time.time() < hold_end
        ):

            self.check_runtime()
            rospy.sleep(0.05)


    # ========================================================
    # main
    # ========================================================

    def run(self):

        self.wait_stack()

        rospy.set_param(
            "/contest2/uav2_touchdown/ground_z_reference",
            self.oz
        )

        self.wait_manual_arm()

        print()
        print("========================================")
        print(" TAKEOFF")
        print("========================================")

        self.run_flight_ops(
            "takeoff"
        )

        self.wait_takeoff_settle()

        for i, wp in enumerate(
            self.waypoints
        ):

            # WAYPOINT_POLE_PRELOOK_V1
            # Before planning P1 -> P2, rotate the forward-facing
            # D435 toward P2 so that the pole is already inside
            # the local map.
            if i == 1:
                self.prelook_to_waypoint(wp)

            self.execute_leg(
                wp,
                settle_after=0.8
            )

        print()
        print("========================================")
        print(" P3 COMPLETE -> LAND")
        print("========================================")

        self.run_flight_ops(
            "land"
        )

        print()
        print("========================================")
        print(" WAYPOINT-POLE DEMO COMPLETE")
        print("========================================")


if __name__ == "__main__":

    rospy.init_node(
        "uav2_waypoint_pole_mission"
    )

    node = WaypointPoleMission()

    try:
        node.run()

    except Exception as e:

        rospy.logerr(
            "MISSION FAILED: %s",
            str(e)
        )

        try:
            node.safe_hold()
        except Exception:
            pass

        raise
