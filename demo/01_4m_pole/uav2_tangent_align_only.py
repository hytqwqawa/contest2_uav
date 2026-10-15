#!/usr/bin/env python3

import math
import time
from pathlib import Path

import rospy
import yaml

from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class AlignOnly:

    def __init__(self):

        self.root = (
            Path(__file__).resolve().parent
        )

        self.target_file = (
            self.root
            / "alignment_target.yaml"
        )

        self.state = None
        self.odom = None
        self.cmd = None

        self.health = False
        self.mission = False
        self.demo_safe = False

        self.point_status = ""
        self.mux_status = ""

        rospy.Subscriber(
            "/contest2/mavros/state",
            State,
            self.state_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/mavros/local_position/odom",
            Odometry,
            self.odom_cb,
            queue_size=50
        )

        rospy.Subscriber(
            "/contest2/position_cmd",
            PositionCommand,
            self.cmd_cb,
            queue_size=100
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/flight_healthy",
            Bool,
            self.health_cb,
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


    def state_cb(self, msg):
        self.state = msg


    def odom_cb(self, msg):
        self.odom = msg


    def cmd_cb(self, msg):
        self.cmd = msg


    def health_cb(self, msg):
        self.health = msg.data


    def mission_cb(self, msg):
        self.mission = msg.data


    def safe_cb(self, msg):
        self.demo_safe = msg.data


    def point_cb(self, msg):
        self.point_status = msg.data


    def mux_cb(self, msg):
        self.mux_status = msg.data


    def current_yaw(self):

        q = self.odom.pose.pose.orientation

        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )


    def call_service(self, name):

        rospy.wait_for_service(
            name,
            timeout=3.0
        )

        cli = rospy.ServiceProxy(
            name,
            Trigger
        )

        result = cli()

        rospy.logwarn(
            "%s -> success=%s | %s",
            name,
            str(result.success),
            result.message
        )

        if not result.success:
            raise RuntimeError(
                "%s rejected: %s"
                % (name, result.message)
            )


    def restore_point_hold(self):

        rospy.logwarn(
            "Restoring POINT HOLD..."
        )

        for name in [
            "/contest2/ego_uav_interface/disable_trajectory",
            "/contest2/ego_uav_interface/abort_trajectory",
            "/contest2/ego_uav_interface/use_point",
        ]:
            try:
                self.call_service(name)
            except Exception as e:
                rospy.logerr(
                    "%s restore failed: %s",
                    name,
                    str(e)
                )

        try:
            self.call_service(
                "/contest2/ego_uav_interface/enable_trajectory"
            )
        except Exception as e:
            rospy.logerr(
                "enable POINT output failed: %s",
                str(e)
            )


    def wait_inputs(self):

        deadline = time.time() + 5.0

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
        ):

            if (
                self.state is not None
                and self.odom is not None
                and self.point_status != ""
                and self.mux_status != ""
            ):
                return

            rospy.sleep(0.05)

        raise RuntimeError(
            "required startup inputs unavailable: "
            "state=%s odom=%s point_status=%r mux_status=%r"
            %
            (
                self.state is not None,
                self.odom is not None,
                self.point_status,
                self.mux_status
            )
        )


    def check_preconditions(self):

        if not self.state.armed:
            raise RuntimeError(
                "vehicle is not ARMED"
            )

        if self.state.mode != "OFFBOARD":
            raise RuntimeError(
                "PX4 is not OFFBOARD"
            )

        if not self.health:
            raise RuntimeError(
                "flight_healthy=False"
            )

        if not self.mission:
            raise RuntimeError(
                "mission permission=False"
            )

        if not self.demo_safe:
            raise RuntimeError(
                "Demo-frame guard UNSAFE"
            )

        if self.point_status != "AIRBORNE_HOLD":
            raise RuntimeError(
                "POINT status=%s"
                % self.point_status
            )

        if self.mux_status != "POINT_ACTIVE":
            raise RuntimeError(
                "MUX status=%s"
                % self.mux_status
            )


    def run(self):

        print()
        print("========================================")
        print(" UAV2 TANGENT ALIGN-ONLY")
        print("========================================")
        print()
        print("This test:")
        print("  YES: rotate yaw in place")
        print("  YES: hold XYZ")
        print()
        print("  NO : traj_start_trigger")
        print("  NO : EGO translation")
        print("  NO : software ARM")
        print()

        self.wait_inputs()
        self.check_preconditions()

        if not self.target_file.exists():
            raise RuntimeError(
                "alignment_target.yaml missing"
            )

        with open(self.target_file, "r") as f:
            target = yaml.safe_load(f)

        traj_id = int(
            target["traj_id"]
        )

        yaw_target_deg = float(
            target["execution_initial_yaw_deg"]
        )

        target_age = (
            time.time()
            -
            float(target["created_wall_time"])
        )

        if target_age > 120.0:
            raise RuntimeError(
                "alignment target stale: %.1f s"
                % target_age
            )

        print(
            "traj_id        = %d"
            % traj_id
        )

        print(
            "yaw target     = %.2f deg"
            % yaw_target_deg
        )

        print(
            "target age     = %.1f s"
            % target_age
        )

        # ------------------------------------------------------
        # SAME-TRAJECTORY handshake
        # ------------------------------------------------------

        deadline = time.time() + 5.0

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
        ):

            if (
                self.cmd is not None
                and
                int(self.cmd.trajectory_id)
                == traj_id
            ):
                break

            rospy.sleep(0.02)

        if (
            self.cmd is None
            or
            int(self.cmd.trajectory_id)
            != traj_id
        ):
            raise RuntimeError(
                "no PositionCommand for traj_id=%d"
                % traj_id
            )

        p = self.odom.pose.pose.position

        hold_x = p.x
        hold_y = p.y
        hold_z = p.z

        first_gap_xy = math.hypot(
            self.cmd.position.x - hold_x,
            self.cmd.position.y - hold_y
        )

        first_gap_z = abs(
            self.cmd.position.z - hold_z
        )

        print(
            "first gap      = "
            "dXY %.3f m  dZ %.3f m"
            %
            (
                first_gap_xy,
                first_gap_z
            )
        )

        if (
            first_gap_xy > 0.08
            or
            first_gap_z > 0.08
        ):
            raise RuntimeError(
                "first gap too large"
            )

        yaw0 = self.current_yaw()

        print(
            "current yaw    = %.2f deg"
            % math.degrees(yaw0)
        )

        print(
            "initial error  = %.2f deg"
            %
            math.degrees(
                wrap(
                    math.radians(
                        yaw_target_deg
                    ) - yaw0
                )
            )
        )

        # ======================================================
        # Proven Plan-Align ordering:
        #
        # freeze output
        # -> FIXED=current
        # -> mux EGO
        # -> enable
        # -> FIXED=target
        #
        # No traj_start_trigger anywhere in this node.
        # ======================================================

        self.call_service(
            "/contest2/ego_uav_interface/disable_trajectory"
        )

        # Prevent a yaw step when EGO source is enabled.
        self.yaw_target_pub.publish(
            Float64(
                data=math.degrees(yaw0)
            )
        )

        self.yaw_mode_pub.publish(
            String(data="FIXED")
        )

        rospy.sleep(0.15)

        self.call_service(
            "/contest2/ego_uav_interface/use_ego"
        )

        deadline = time.time() + 1.5

        while (
            time.time() < deadline
            and
            self.mux_status != "EGO_ACTIVE"
        ):
            rospy.sleep(0.02)

        if self.mux_status != "EGO_ACTIVE":
            raise RuntimeError(
                "MUX did not enter EGO_ACTIVE"
            )

        self.call_service(
            "/contest2/ego_uav_interface/enable_trajectory"
        )

        # ALIGN_HANDOVER_REANCHOR_V1
        #
        # At this point:
        #   - EGO mux is active
        #   - FIXED translation is already holding actual odom XYZ
        #   - yaw target is STILL the current aircraft yaw
        #
        # Do not count the POINT -> EGO handover transient
        # against the 0.12-m yaw-alignment drift budget.
        rospy.sleep(0.50)

        p = self.odom.pose.pose.position

        hold_x = p.x
        hold_y = p.y
        hold_z = p.z

        rospy.logwarn(
            "[ALIGN] drift reference re-anchored "
            "after FIXED handover: "
            "(%.3f, %.3f, %.3f)",
            hold_x,
            hold_y,
            hold_z
        )

        self.yaw_target_pub.publish(
            Float64(
                data=yaw_target_deg
            )
        )

        self.yaw_mode_pub.publish(
            String(data="FIXED")
        )

        print()
        print("========================================")
        print(" ALIGN START")
        print("========================================")
        print(
            "hold XYZ  = (%.3f, %.3f, %.3f)"
            %
            (
                hold_x,
                hold_y,
                hold_z
            )
        )

        print(
            "target yaw = %.2f deg"
            % yaw_target_deg
        )

        print(
            "NO /contest2/traj_start_trigger"
        )
        print()

        yaw_tol = math.radians(8.0)

        stable_required = 0.8

        stable_since = None

        max_xy = 0.0
        max_z = 0.0

        deadline = time.time() + 20.0

        success = False

        rate = rospy.Rate(20)

        while (
            not rospy.is_shutdown()
            and
            time.time() < deadline
        ):

            if (
                self.state is None
                or
                not self.state.armed
                or
                self.state.mode != "OFFBOARD"
                or
                not self.health
                or
                not self.mission
                or
                not self.demo_safe
            ):
                raise RuntimeError(
                    "runtime safety condition lost"
                )

            if self.mux_status != "EGO_ACTIVE":
                raise RuntimeError(
                    "MUX left EGO_ACTIVE"
                )

            p = self.odom.pose.pose.position

            dxy = math.hypot(
                p.x - hold_x,
                p.y - hold_y
            )

            dz = abs(
                p.z - hold_z
            )

            max_xy = max(
                max_xy,
                dxy
            )

            max_z = max(
                max_z,
                dz
            )

            if (
                dxy > 0.12
                or
                dz > 0.12
            ):
                raise RuntimeError(
                    "XYZ drift too large: "
                    "dXY=%.3f dZ=%.3f"
                    %
                    (
                        dxy,
                        dz
                    )
                )

            yaw = self.current_yaw()

            err = wrap(
                math.radians(
                    yaw_target_deg
                )
                -
                yaw
            )

            if abs(err) <= yaw_tol:

                if stable_since is None:
                    stable_since = time.time()

                stable = (
                    time.time()
                    -
                    stable_since
                )

            else:

                stable_since = None
                stable = 0.0

            print(
                "\rALIGN "
                "yaw=%+7.2f "
                "target=%+7.2f "
                "err=%+6.2f deg "
                "dXY=%.3f "
                "dZ=%.3f "
                "stable=%.2f/%.2f"
                %
                (
                    math.degrees(yaw),
                    yaw_target_deg,
                    math.degrees(err),
                    dxy,
                    dz,
                    stable,
                    stable_required
                ),
                end="",
                flush=True
            )

            if stable >= stable_required:
                success = True
                break

            rate.sleep()

        print()
        print()

        if not success:
            raise RuntimeError(
                "yaw alignment timeout"
            )

        final_yaw = self.current_yaw()

        final_err = math.degrees(
            wrap(
                math.radians(
                    yaw_target_deg
                )
                -
                final_yaw
            )
        )

        result = (
            "traj_id=%d\n"
            "target_yaw_deg=%.3f\n"
            "actual_yaw_deg=%.3f\n"
            "final_error_deg=%.3f\n"
            "max_xy_drift=%.4f\n"
            "max_z_drift=%.4f\n"
            "traj_start_trigger_sent=False\n"
            %
            (
                traj_id,
                yaw_target_deg,
                math.degrees(final_yaw),
                final_err,
                max_xy,
                max_z
            )
        )

        (
            self.root
            / "alignment_result.txt"
        ).write_text(result)

        print("========================================")
        print(" ALIGN PASSED")
        print("========================================")
        print(result)
        print(
            "Holding aligned attitude for 2 s..."
        )

        rospy.sleep(2.0)

        self.restore_point_hold()

        print()
        print("[ OK ] returned to POINT HOLD")


if __name__ == "__main__":

    rospy.init_node(
        "uav2_tangent_align_only"
    )

    node = AlignOnly()

    try:
        node.run()

    except Exception as e:

        rospy.logerr(
            "ALIGN-ONLY FAILED: %s",
            str(e)
        )

        try:
            node.restore_point_hold()
        except Exception:
            pass

        raise
