#!/usr/bin/env python3

import math
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from mavros_msgs.msg import PositionTarget
from quadrotor_msgs.msg import PositionCommand
from tf.transformations import euler_from_quaternion


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class EgoTrajectoryBridge:

    def __init__(self):

        self.yaw_rate_limit = rospy.get_param(
            "~yaw_rate_limit", 0.50
        )

        self.max_yaw_lead = rospy.get_param(
            "~max_yaw_lead", 0.20
        )

        self.min_tangent_speed = rospy.get_param(
            "~min_tangent_speed", 0.06
        )

        self.current_yaw = None
        self.current_position = None
        self.yaw_cmd = None

        self.tangent_yaw = None
        self.hold_yaw = None
        self.fixed_yaw = None

        # Default behaviour for ordinary RViz tests.
        self.yaw_mode = "TANGENT"

        self.last_time = None
        self.active_traj_id = None
        self.mux_ego_active = False

        self.output_pub = rospy.Publisher(
            "/contest1/ego_uav_interface/ego_setpoint",
            PositionTarget,
            queue_size=50
        )

        self.status_pub = rospy.Publisher(
            "/contest1/ego_uav_interface/ego_bridge_status",
            String,
            queue_size=1,
            latch=True
        )

        rospy.Subscriber(
            "/contest1/position_cmd",
            PositionCommand,
            self.command_cb,
            queue_size=100
        )

        rospy.Subscriber(
            "/contest1/mavros/local_position/odom",
            Odometry,
            self.odom_cb,
            queue_size=50
        )

        rospy.Subscriber(
            "/contest1/ego_uav_interface/trajectory_mux_status",
            String,
            self.mux_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest1/ego_uav_interface/yaw_mode",
            String,
            self.yaw_mode_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest1/ego_uav_interface/yaw_target_deg",
            Float64,
            self.yaw_target_cb,
            queue_size=10
        )

        self.status_pub.publish(
            String(data="READY yaw_mode=TANGENT mask=2048")
        )

        rospy.loginfo("======================================")
        rospy.loginfo(" EGO MULTI-MODE YAW BRIDGE")
        rospy.loginfo("--------------------------------------")
        rospy.loginfo(" HOLD    : keep current heading")
        rospy.loginfo(" TANGENT : follow trajectory tangent")
        rospy.loginfo(" FIXED   : rotate to fixed yaw")
        rospy.loginfo(" mask    : 2048")
        rospy.loginfo("======================================")


    def odom_cb(self, msg):

        q = msg.pose.pose.orientation

        _, _, yaw = euler_from_quaternion([
            q.x,
            q.y,
            q.z,
            q.w
        ])

        self.current_yaw = yaw

        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

        if self.yaw_cmd is None:
            self.yaw_cmd = yaw


    def mux_cb(self, msg):

        self.mux_ego_active = (
            msg.data == "EGO_ACTIVE"
        )

        if (
            not self.mux_ego_active
            and self.current_yaw is not None
        ):

            self.yaw_cmd = self.current_yaw
            self.last_time = rospy.Time.now()

            # HOLD should capture the actual aircraft heading
            # immediately before EGO takes control.
            if self.yaw_mode == "HOLD":
                self.hold_yaw = self.current_yaw


    def yaw_mode_cb(self, msg):

        mode = msg.data.strip().upper()

        if mode not in [
            "HOLD",
            "TANGENT",
            "FIXED"
        ]:
            rospy.logwarn(
                "[YAW MODE] invalid mode: %s",
                mode
            )
            return

        self.yaw_mode = mode

        if self.current_yaw is not None:

            if mode == "HOLD":

                self.hold_yaw = self.current_yaw
                self.yaw_cmd = self.current_yaw

            elif mode == "FIXED":

                # Start fixed-yaw rotation from real aircraft yaw.
                self.yaw_cmd = self.current_yaw

        self.last_time = rospy.Time.now()

        rospy.logwarn(
            "[YAW MODE] -> %s",
            self.yaw_mode
        )


    def yaw_target_cb(self, msg):

        self.fixed_yaw = math.radians(
            msg.data
        )

        self.fixed_yaw = wrap_angle(
            self.fixed_yaw
        )

        rospy.logwarn(
            "[YAW TARGET] %.1f deg",
            msg.data
        )


    def command_cb(self, msg):

        if self.current_yaw is None:
            return

        now = rospy.Time.now()

        if msg.trajectory_id != self.active_traj_id:

            self.active_traj_id = msg.trajectory_id

            self.yaw_cmd = self.current_yaw
            self.last_time = now

            rospy.logwarn(
                "[EGO BRIDGE] new trajectory id=%d",
                self.active_traj_id
            )

        vx = msg.velocity.x
        vy = msg.velocity.y

        vxy = math.hypot(vx, vy)

        if vxy >= self.min_tangent_speed:

            self.tangent_yaw = math.atan2(
                vy,
                vx
            )

        # ====================================================
        # Desired yaw according to current mission mode
        # ====================================================

        if self.yaw_mode == "HOLD":

            if self.hold_yaw is None:
                self.hold_yaw = self.current_yaw

            desired_yaw = self.hold_yaw


        elif self.yaw_mode == "FIXED":

            if self.fixed_yaw is None:
                desired_yaw = self.current_yaw
            else:
                desired_yaw = self.fixed_yaw


        else:  # TANGENT

            if self.tangent_yaw is None:
                desired_yaw = self.current_yaw
            else:
                desired_yaw = self.tangent_yaw


        # Before EGO mux takes control, never rotate in background.
        if not self.mux_ego_active:

            self.yaw_cmd = self.current_yaw
            self.last_time = now

        else:

            if self.last_time is None:
                self.last_time = now

            dt = (
                now - self.last_time
            ).to_sec()

            dt = max(
                0.001,
                min(dt, 0.05)
            )

            self.last_time = now

            err = wrap_angle(
                desired_yaw - self.yaw_cmd
            )

            max_step = (
                self.yaw_rate_limit * dt
            )

            step = max(
                -max_step,
                min(max_step, err)
            )

            proposed = wrap_angle(
                self.yaw_cmd + step
            )

            # Never let command yaw get too far ahead of
            # actual aircraft yaw.
            lead = wrap_angle(
                proposed - self.current_yaw
            )

            if abs(lead) > self.max_yaw_lead:

                proposed = wrap_angle(
                    self.current_yaw
                    +
                    math.copysign(
                        self.max_yaw_lead,
                        lead
                    )
                )

            self.yaw_cmd = proposed


        # ====================================================
        # FIXED_TRANSLATION_HOLD_V4
        #
        # FIXED = true in-place yaw alignment.
        #
        # On entry, capture the currently received trajectory
        # position. While FIXED:
        #
        #   P = captured XYZ
        #   V = 0
        #   A = 0
        #   yaw = yaw_target through the normal yaw channel
        #
        # For P2 this mode is entered before the preview goal,
        # therefore the captured point is the final P1 hold.
        # ====================================================

        if not hasattr(
            self,
            "_fixed_prev_active"
        ):
            self._fixed_prev_active = False
            self._fixed_hold_xyz = None

        fixed_active = (
            self.yaw_mode == "FIXED"
        )

        if (
            fixed_active
            and not self._fixed_prev_active
        ):

            if self.current_position is None:
                rospy.logerr(
                    "[EGO BRIDGE] cannot enter FIXED: "
                    "actual odom XYZ unavailable"
                )
                return

	    try:
		  z_ref = float(rospy.get_param(
		      "/contest2/ego_uav_interface/terminal_yaw_hold_z"
		  ))
	    except (KeyError, TypeError, ValueError):
		  rospy.logerr("[FIXED] target height unavailable")
		  return

	    if not math.isfinite(z_ref):
		  rospy.logerr("[FIXED] target height invalid")
		  return

	    self._fixed_hold_xyz = (
		  self.current_position[0],
		  self.current_position[1],
		  z_ref
	    )

            rospy.logwarn(
                "[EGO BRIDGE] FIXED ODOM XYZ captured: "
                "(%.3f, %.3f, %.3f)",
                self._fixed_hold_xyz[0],
                self._fixed_hold_xyz[1],
                self._fixed_hold_xyz[2]
            )

        if not fixed_active:
            self._fixed_hold_xyz = None

        self._fixed_prev_active = (
            fixed_active
        )


        # ====================================================
        # MAVROS PositionTarget
        # P + V + A + yaw
        # yaw_rate ignored
        # ====================================================

        out = PositionTarget()

        out.header.stamp = now
        out.header.frame_id = "map"

        out.coordinate_frame = (
            PositionTarget.FRAME_LOCAL_NED
        )

        out.type_mask = (
            PositionTarget.IGNORE_YAW_RATE
        )

        if (
            self.yaw_mode == "FIXED"
            and self._fixed_hold_xyz is not None
        ):

            out.position.x = (
                self._fixed_hold_xyz[0]
            )

            out.position.y = (
                self._fixed_hold_xyz[1]
            )

            out.position.z = (
                self._fixed_hold_xyz[2]
            )

            out.velocity.x = 0.0
            out.velocity.y = 0.0
            out.velocity.z = 0.0

            out.acceleration_or_force.x = 0.0
            out.acceleration_or_force.y = 0.0
            out.acceleration_or_force.z = 0.0

        else:

            out.position.x = msg.position.x
            out.position.y = msg.position.y
            out.position.z = msg.position.z

            out.velocity.x = msg.velocity.x
            out.velocity.y = msg.velocity.y
            out.velocity.z = msg.velocity.z

            out.acceleration_or_force.x = (
                msg.acceleration.x
            )

            out.acceleration_or_force.y = (
                msg.acceleration.y
            )

            out.acceleration_or_force.z = (
                msg.acceleration.z
            )

        out.yaw = self.yaw_cmd
        out.yaw_rate = 0.0

        self.output_pub.publish(out)


        if not hasattr(self, "_last_status"):
            self._last_status = rospy.Time(0)

        if (
            now - self._last_status
        ).to_sec() > 0.5:

            tangent_deg = (
                math.degrees(self.tangent_yaw)
                if self.tangent_yaw is not None
                else float("nan")
            )

            self.status_pub.publish(
                String(
                    data=(
                        "mode=%s "
                        "traj=%s "
                        "vxy=%.2f "
                        "actual=%.1f "
                        "tangent=%.1f "
                        "cmd=%.1f"
                        %
                        (
                            self.yaw_mode,
                            str(self.active_traj_id),
                            vxy,
                            math.degrees(
                                self.current_yaw
                            ),
                            tangent_deg,
                            math.degrees(
                                self.yaw_cmd
                            )
                        )
                    )
                )
            )

            self._last_status = now


if __name__ == "__main__":

    rospy.init_node(
        "ego_trajectory_bridge"
    )

    EgoTrajectoryBridge()

    rospy.spin()
