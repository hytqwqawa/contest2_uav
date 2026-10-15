#!/usr/bin/env python3

import math
import copy
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse
from mavros_msgs.msg import PositionTarget
from tf.transformations import euler_from_quaternion


class TrajectoryInterface:

    def __init__(self):

        self.odom_topic = rospy.get_param(
            "~odom_topic",
            "/contest2/mavros/local_position/odom"
        )

        self.health_topic = rospy.get_param(
            "~health_topic",
            "/contest2/ego_uav_interface/flight_healthy"
        )

        self.command_topic = rospy.get_param(
            "~command_topic",
            "/contest2/ego_uav_interface/trajectory_setpoint"
        )

        self.mission_permission_topic = rospy.get_param(
            "~mission_permission_topic",
            "/contest2/ego_uav_interface/rc_mission_allowed"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/contest2/mavros/setpoint_raw/local"
        )

        self.publish_rate = rospy.get_param(
            "~publish_rate", 50.0
        )

        self.odom_timeout = rospy.get_param(
            "~odom_timeout", 0.20
        )

        self.health_timeout = rospy.get_param(
            "~health_timeout", 0.30
        )

        self.command_timeout = rospy.get_param(
            "~command_timeout", 0.20
        )

        self.mission_permission_timeout = rospy.get_param(
            "~mission_permission_timeout", 1.50
        )

        self.stream_ready_time = rospy.get_param(
            "~stream_ready_time", 1.50
        )

        # Geofence
        self.fence_x_min = rospy.get_param("~fence_x_min", -3.0)
        self.fence_x_max = rospy.get_param("~fence_x_max",  3.5)
        self.fence_y_min = rospy.get_param("~fence_y_min", -1.7)
        self.fence_y_max = rospy.get_param("~fence_y_max",  1.7)

        # Planner must stay this far inside the hard fence.
        self.fence_margin = rospy.get_param("~fence_margin", 0.20)

        self.odom = None
        self.odom_time = None

        self.flight_healthy = False
        self.health_time = None

        self.traj_cmd = None
        self.traj_time = None
        self.trajectory_enabled = False

        # Mission permission from rc_supervisor.
        #
        # Fail-safe default is FALSE. A fresh TRUE is required
        # before trajectory input may be enabled.
        self.mission_permission = False
        self.mission_permission_time = None

        self.hold_valid = False
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z = 0.0
        self.hold_yaw = 0.0

        self.last_source = "NONE"

        self.stream_start_time = None
        self.stream_ready = False

        self.geofence_blocked = False
        self.geofence_recovery = False

        rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=20
        )

        rospy.Subscriber(
            self.health_topic,
            Bool,
            self.health_callback,
            queue_size=20
        )

        rospy.Subscriber(
            self.command_topic,
            PositionTarget,
            self.command_callback,
            queue_size=20
        )

        rospy.Subscriber(
            self.mission_permission_topic,
            Bool,
            self.mission_permission_callback,
            queue_size=20
        )

        self.output_pub = rospy.Publisher(
            self.output_topic,
            PositionTarget,
            queue_size=10
        )

        self.ready_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/offboard_stream_ready",
            Bool,
            queue_size=1,
            latch=True
        )

        self.status_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/trajectory_interface_status",
            String,
            queue_size=1,
            latch=True
        )

        self.geofence_ok_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/geofence_ok",
            Bool,
            queue_size=1,
            latch=True
        )

        self.geofence_status_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/geofence_status",
            String,
            queue_size=1,
            latch=True
        )

        self.enable_srv = rospy.Service(
            "/contest2/ego_uav_interface/enable_trajectory",
            Trigger,
            self.enable_trajectory
        )

        self.disable_srv = rospy.Service(
            "/contest2/ego_uav_interface/disable_trajectory",
            Trigger,
            self.disable_trajectory
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self.timer_callback
        )

        rospy.loginfo("======================================")
        rospy.loginfo(" trajectory_interface.py")
        rospy.loginfo("--------------------------------------")
        rospy.loginfo(" Odom       : %s", self.odom_topic)
        rospy.loginfo(" Health     : %s", self.health_topic)
        rospy.loginfo(" Command    : %s", self.command_topic)
        rospy.loginfo(" MAVROS out : %s", self.output_topic)
        rospy.loginfo(" Rate       : %.1f Hz", self.publish_rate)
        rospy.loginfo("--------------------------------------")
        rospy.loginfo(" Default source: HOLD")
        rospy.loginfo(" Trajectory requires explicit enable")
        rospy.loginfo(" Trajectory also requires fresh RC mission permission")
        rospy.loginfo(" DOES NOT ARM")
        rospy.loginfo(" DOES NOT CHANGE PX4 MODE")
        rospy.loginfo("======================================")

    def odom_callback(self, msg):
        self.odom = msg
        self.odom_time = rospy.Time.now()

    def health_callback(self, msg):
        self.flight_healthy = msg.data
        self.health_time = rospy.Time.now()

    def command_callback(self, msg):
        self.traj_cmd = msg
        self.traj_time = rospy.Time.now()

    def mission_permission_callback(self, msg):

        self.mission_permission = msg.data
        self.mission_permission_time = rospy.Time.now()

        # CH7 leaving HIGH is also the reset action after
        # a geofence-aborted mission. Never resume automatically.
        if (
            not msg.data
            and self.geofence_blocked
            and self.odom is not None
            and self.current_inside_planning_fence()
        ):
            self.geofence_blocked = False
            self.geofence_recovery = False
            self.capture_hold()

            rospy.logwarn(
                "[GEOFENCE] block latch reset by mission permission OFF"
            )

    def odom_fresh(self):
        return (
            self.odom_time is not None and
            (rospy.Time.now() - self.odom_time).to_sec()
            < self.odom_timeout
        )

    def health_ok(self):
        return (
            self.health_time is not None and
            self.flight_healthy and
            (rospy.Time.now() - self.health_time).to_sec()
            < self.health_timeout
        )

    def trajectory_fresh(self):
        return (
            self.traj_time is not None and
            (rospy.Time.now() - self.traj_time).to_sec()
            < self.command_timeout
        )

    def mission_permission_fresh(self):
        return (
            self.mission_permission_time is not None and
            (
                rospy.Time.now() - self.mission_permission_time
            ).to_sec() < self.mission_permission_timeout
        )

    def mission_permission_ok(self):
        return (
            self.mission_permission_fresh() and
            self.mission_permission
        )

    def inside_hard_fence(self, x, y):

        return (
            self.fence_x_min <= x <= self.fence_x_max
            and
            self.fence_y_min <= y <= self.fence_y_max
        )

    def inside_planning_fence(self, x, y):

        return (
            self.fence_x_min + self.fence_margin
            <= x <=
            self.fence_x_max - self.fence_margin
            and
            self.fence_y_min + self.fence_margin
            <= y <=
            self.fence_y_max - self.fence_margin
        )

    def current_inside_planning_fence(self):

        if self.odom is None:
            return False

        p = self.odom.pose.pose.position

        return self.inside_planning_fence(
            p.x,
            p.y
        )

    def make_geofence_recovery_command(self):

        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation

        _, _, yaw = euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])

        xmin = self.fence_x_min + self.fence_margin
        xmax = self.fence_x_max - self.fence_margin
        ymin = self.fence_y_min + self.fence_margin
        ymax = self.fence_y_max - self.fence_margin

        tx = min(max(p.x, xmin), xmax)
        ty = min(max(p.y, ymin), ymax)

        cmd = PositionTarget()

        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "map"

        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Conservative position-only recovery.
        cmd.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        cmd.position.x = tx
        cmd.position.y = ty
        cmd.position.z = p.z
        cmd.yaw = yaw

        return cmd

    def capture_hold(self):

        if self.odom is None:
            return False

        self.hold_x = self.odom.pose.pose.position.x
        self.hold_y = self.odom.pose.pose.position.y
        self.hold_z = self.odom.pose.pose.position.z

        q = self.odom.pose.pose.orientation

        _, _, self.hold_yaw = euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])

        self.hold_valid = True

        rospy.logwarn(
            "HOLD captured: [%.3f %.3f %.3f], yaw=%.2f deg",
            self.hold_x,
            self.hold_y,
            self.hold_z,
            math.degrees(self.hold_yaw)
        )

        return True

    def make_hold_command(self):

        cmd = PositionTarget()

        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "map"

        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        cmd.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        cmd.position.x = self.hold_x
        cmd.position.y = self.hold_y
        cmd.position.z = self.hold_z

        cmd.yaw = self.hold_yaw

        return cmd

    def make_trajectory_command(self):

        cmd = copy.deepcopy(self.traj_cmd)

        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "map"

        # Preserve the upstream field-validity mask.
        # MAVROS will perform the ROS ENU -> MAVLink NED conversion.
        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # IMPORTANT:
        # Do NOT force type_mask=0 here.
        # Each trajectory source decides which fields are active.

        return cmd

    def enable_trajectory(self, req):

        if self.geofence_blocked:

            return TriggerResponse(
                success=False,
                message="Rejected: geofence mission latch is active"
            )

        if not self.current_inside_planning_fence():

            return TriggerResponse(
                success=False,
                message="Rejected: vehicle outside planning geofence"
            )

        if not self.mission_permission_fresh():

            return TriggerResponse(
                success=False,
                message="Rejected: RC mission permission unavailable/stale"
            )

        if not self.mission_permission:

            return TriggerResponse(
                success=False,
                message="Rejected: RC mission permission is false"
            )

        self.trajectory_enabled = True

        return TriggerResponse(
            success=True,
            message="Trajectory input enabled"
        )

    def disable_trajectory(self, req):

        self.trajectory_enabled = False

        if self.odom_fresh():
            self.capture_hold()

        return TriggerResponse(
            success=True,
            message="Trajectory input disabled; HOLD active"
        )

    def reset_stream(self):

        self.stream_ready = False
        self.stream_start_time = None
        self.hold_valid = False
        self.last_source = "NONE"

    def timer_callback(self, event):

        now = rospy.Time.now()

        if not self.odom_fresh():

            self.reset_stream()

            self.ready_pub.publish(False)
            self.status_pub.publish("WAIT_ODOM")

            rospy.logwarn_throttle(
                1.0,
                "[TRAJ IF] WAIT_ODOM"
            )
            return

        if not self.health_ok():

            self.reset_stream()

            self.ready_pub.publish(False)
            self.status_pub.publish("WAIT_FLIGHT_HEALTHY")

            rospy.logwarn_throttle(
                1.0,
                "[TRAJ IF] WAIT_FLIGHT_HEALTHY"
            )
            return

        # ==================================================
        # GEOFENCE - final safety layer before MAVROS
        # ==================================================

        pos = self.odom.pose.pose.position

        inside_hard = self.inside_hard_fence(
            pos.x,
            pos.y
        )

        inside_plan = self.inside_planning_fence(
            pos.x,
            pos.y
        )

        # Actual vehicle crossed the HARD fence:
        # abort everything and drive toward the nearest point
        # inside the planning fence.
        if not inside_hard:

            if not self.geofence_recovery:

                rospy.logerr(
                    "[GEOFENCE] HARD FENCE VIOLATION "
                    "x=%.3f y=%.3f -> RECOVERY",
                    pos.x,
                    pos.y
                )

            self.geofence_recovery = True
            self.geofence_blocked = True

            self.trajectory_enabled = False
            self.traj_cmd = None
            self.traj_time = None

        # Once recovery begins, continue until the vehicle is
        # fully back inside the INNER planning fence.
        if self.geofence_recovery:

            if inside_plan:

                self.geofence_recovery = False
                self.capture_hold()

                rospy.logwarn(
                    "[GEOFENCE] vehicle recovered -> HOLD. "
                    "Old mission remains cancelled."
                )

            else:

                cmd = self.make_geofence_recovery_command()

                self.output_pub.publish(cmd)

                if self.stream_start_time is None:
                    self.stream_start_time = now

                self.stream_ready = (
                    (now - self.stream_start_time).to_sec()
                    >= self.stream_ready_time
                )

                self.ready_pub.publish(
                    Bool(data=self.stream_ready)
                )

                self.geofence_ok_pub.publish(
                    Bool(data=False)
                )

                self.geofence_status_pub.publish(
                    String(data="HARD_VIOLATION_RECOVERY")
                )

                self.status_pub.publish(
                    String(data="GEOFENCE_RECOVERY")
                )

                self.last_source = "GEOFENCE_RECOVERY"
                return

        # Latch remains active after a rejected mission.
        geofence_ok = (
            inside_plan
            and not self.geofence_blocked
        )

        self.geofence_ok_pub.publish(
            Bool(data=geofence_ok)
        )

        if self.geofence_blocked:
            self.geofence_status_pub.publish(
                String(data="MISSION_BLOCKED_HOLD")
            )
        elif inside_plan:
            self.geofence_status_pub.publish(
                String(data="SAFE")
            )
        else:
            self.geofence_status_pub.publish(
                String(data="BUFFER_ZONE")
            )

        mission_ok = self.mission_permission_ok()

        # --------------------------------------------------
        # RC mission permission is a hard trajectory gate.
        #
        # If an active trajectory loses permission, capture
        # the CURRENT measured position and immediately return
        # to HOLD.
        #
        # This covers:
        #   CH7 HIGH -> MID/LOW
        #   CH6 manual takeover latch
        #   rc_supervisor crash / RC permission timeout
        # --------------------------------------------------

        if self.trajectory_enabled and not mission_ok:

            self.trajectory_enabled = False

            self.capture_hold()

            # Do not let the previous command remain selected
            # internally after RC permission is revoked.
            self.traj_cmd = None
            self.traj_time = None

            # HOLD was captured above, so avoid recapturing it
            # again below merely because last_source used to
            # be TRAJECTORY.
            self.last_source = "HOLD"

            rospy.logerr(
                "[TRAJ IF] RC mission permission lost. "
                "Trajectory disabled -> HOLD CURRENT POSITION."
            )

        use_trajectory = (
            self.trajectory_enabled and
            mission_ok and
            self.trajectory_fresh()
            and not self.geofence_blocked
        )

        # Any planner / point / future EGO command entering the
        # inner forbidden buffer cancels the entire current task.
        if use_trajectory:

            tx = self.traj_cmd.position.x
            ty = self.traj_cmd.position.y

            if not self.inside_planning_fence(tx, ty):

                rospy.logerr(
                    "[GEOFENCE] unsafe command rejected: "
                    "x=%.3f y=%.3f",
                    tx,
                    ty
                )

                self.geofence_blocked = True
                self.trajectory_enabled = False

                self.traj_cmd = None
                self.traj_time = None

                self.capture_hold()
                self.last_source = "HOLD"

                use_trajectory = False

                self.geofence_ok_pub.publish(
                    Bool(data=False)
                )

                self.geofence_status_pub.publish(
                    String(data="UNSAFE_COMMAND_REJECTED")
                )

        if use_trajectory:

            source = "TRAJECTORY"

            if self.last_source != source:
                rospy.logwarn(
                    "Setpoint source -> TRAJECTORY"
                )

            cmd = self.make_trajectory_command()

        else:

            source = "HOLD"

            if not self.hold_valid or self.last_source == "TRAJECTORY":
                self.capture_hold()

                if self.trajectory_enabled:
                    rospy.logwarn(
                        "Trajectory missing/stale -> HOLD fallback"
                    )

            cmd = self.make_hold_command()

        self.output_pub.publish(cmd)

        if self.stream_start_time is None:
            self.stream_start_time = now

        stream_age = (
            now - self.stream_start_time
        ).to_sec()

        self.stream_ready = (
            stream_age >= self.stream_ready_time
        )

        self.ready_pub.publish(
            Bool(data=self.stream_ready)
        )

        if source == "TRAJECTORY":

            status = "TRAJECTORY_ACTIVE"

        elif not self.mission_permission_fresh():

            status = "HOLD_RC_PERMISSION_STALE"

        elif not self.mission_permission:

            status = "HOLD_RC_BLOCKED"

        elif self.trajectory_enabled:

            status = "HOLD_WAIT_TRAJECTORY"

        else:

            status = "HOLD_ACTIVE"

        if not self.stream_ready:
            status = "STREAM_WARMUP_" + status

        self.status_pub.publish(
            String(data=status)
        )

        self.last_source = source

        rospy.loginfo_throttle(
            1.0,
            "[TRAJ IF] source=%s "
            "stream_ready=%s "
            "traj_enabled=%s "
            "traj_fresh=%s "
            "mission_rc=%s",
            source,
            "YES" if self.stream_ready else "NO",
            "YES" if self.trajectory_enabled else "NO",
            "YES" if self.trajectory_fresh() else "NO",
            "ALLOW" if mission_ok else "BLOCK"
        )


if __name__ == "__main__":

    rospy.init_node("trajectory_interface")

    TrajectoryInterface()

    rospy.spin()
