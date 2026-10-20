#!/usr/bin/env python3

import math
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Range
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion


class PointTrajectoryNode:

    def __init__(self):

        # ---------------- parameters ----------------

        self.odom_topic = rospy.get_param(
            "~odom_topic",
            "/contest2/mavros/local_position/odom"
        )

        self.output_topic = rospy.get_param(
            "~output_topic",
            "/contest2/ego_uav_interface/trajectory_setpoint"
        )

        self.mission_permission_topic = rospy.get_param(
            "~mission_permission_topic",
            "/contest2/ego_uav_interface/rc_mission_allowed"
        )

        self.state_topic = rospy.get_param(
            "~state_topic",
            "/contest2/mavros/state"
        )

        self.enable_trajectory_service = rospy.get_param(
            "~enable_trajectory_service",
            "/contest2/ego_uav_interface/enable_trajectory"
        )

        self.offboard_mode = rospy.get_param(
            "~offboard_mode",
            "OFFBOARD"
        )

        self.takeoff_height = rospy.get_param(
            "~takeoff_height",
            0.50
        )

        # Conservative first-flight limits
        self.max_speed = rospy.get_param(
            "~max_speed",
            0.30
        )

        self.max_acc = rospy.get_param(
            "~max_acc",
            0.60
        )

        self.takeoff_speed = rospy.get_param(
            "~takeoff_speed",
            0.40
        )

        self.takeoff_acc = rospy.get_param(
            "~takeoff_acc",
            0.80
        )

        self.land_speed = rospy.get_param(
            "~land_speed",
            0.30
        )

        self.land_acc = rospy.get_param(
            "~land_acc",
            0.60
        )

        # UAV2 terminal descent through ground-effect region.
        self.range_topic = rospy.get_param(
            "~range_topic",
            "/contest2/mavros/distance_sensor/hrlv_ez4_pub"
        )

        # Below this measured range, stop decelerating the
        # quintic landing profile and use a constant sink rate.
        self.land_terminal_range = rospy.get_param(
            "~land_terminal_range",
            0.18
        )

        self.land_terminal_speed = rospy.get_param(
            "~land_terminal_speed",
            0.18
        )

        self.land_terminal_min_offset = rospy.get_param(
            "~land_terminal_min_offset",
            -0.20
        )

        self.range_timeout = rospy.get_param(
            "~range_timeout",
            0.30
        )

        # FINAL_DROP_10CM_V2
        # At 0.30 m/s, a 0.10 m final drop lasts about 0.33 s.
        # Allow a short localization/health dropout during this
        # already-committed touchdown segment, but never indefinitely.
        self.land_terminal_fault_grace = rospy.get_param(
            "~land_terminal_fault_grace",
            0.60
        )

        self.min_duration = rospy.get_param(
            "~min_duration",
            2.5
        )

        self.publish_rate = rospy.get_param(
            "~publish_rate",
            50.0
        )

        self.odom_timeout = 0.20
        self.health_timeout = 0.30

        self.mission_permission_timeout = rospy.get_param(
            "~mission_permission_timeout",
            0.50
        )

        # MAVROS state is only about 1 Hz on the current setup,
        # so this timeout is intentionally larger.
        self.state_timeout = rospy.get_param(
            "~state_timeout",
            2.50
        )

        # Relative height above the startup ground reference
        # used only for mission-state classification.
        self.airborne_height_threshold = rospy.get_param(
            "~airborne_height_threshold",
            0.08
        )

        # ---------------- state ----------------

        self.odom = None
        self.odom_time = None

        self.flight_healthy = False
        self.health_time = None

        self.mission_permission = False
        self.mission_permission_time = None

        self.px4_state = None
        self.state_time = None

        self.initialized = False
        self.takeoff_completed = False

        self.mode = "WAIT_ODOM"

        self.hold_p = [0.0, 0.0, 0.0]
        self.hold_yaw = 0.0

        # Ground reference captured once at startup
        self.home_p = [0.0, 0.0, 0.0]

        self.traj_active = False
        self.traj_kind = ""

        self.traj_t0 = None
        self.traj_T = 1.0

        self.p0 = [0.0, 0.0, 0.0]
        self.pf = [0.0, 0.0, 0.0]

        self.range_value = None
        self.range_time = None

        self.land_terminal_active = False
        self.land_terminal_z_ref = 0.0
        self.land_terminal_xy = [0.0, 0.0]
        self.land_terminal_last_time = None
        self.land_terminal_start_time = None

        # ---------------- ROS ----------------

        rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/flight_healthy",
            Bool,
            self.health_callback,
            queue_size=20
        )

        rospy.Subscriber(
            self.mission_permission_topic,
            Bool,
            self.mission_permission_callback,
            queue_size=20
        )

        rospy.Subscriber(
            self.state_topic,
            State,
            self.state_callback,
            queue_size=20
        )

        rospy.Subscriber(
            self.range_topic,
            Range,
            self.range_callback,
            queue_size=20
        )

        # RViz 2D Nav Goal
        rospy.Subscriber(
            "/contest2/ego_uav_interface/point_goal",
            PoseStamped,
            self.goal_callback,
            queue_size=5
        )

        # Terminal/manual goal
        rospy.Subscriber(
            "/contest2/ego_uav_interface/goal",
            PoseStamped,
            self.goal_callback,
            queue_size=5
        )

        self.cmd_pub = rospy.Publisher(
            self.output_topic,
            PositionTarget,
            queue_size=10
        )

        self.status_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/point_traj_status",
            String,
            queue_size=1,
            latch=True
        )

        rospy.Service(
            "/contest2/ego_uav_interface/start_takeoff",
            Trigger,
            self.start_takeoff
        )

        rospy.Service(
            "/contest2/ego_uav_interface/start_land",
            Trigger,
            self.start_land
        )

        rospy.Service(
            "/contest2/ego_uav_interface/abort_trajectory",
            Trigger,
            self.abort_trajectory
        )

        self.enable_trajectory_client = rospy.ServiceProxy(
            self.enable_trajectory_service,
            Trigger
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self.timer_callback
        )

        rospy.loginfo("==========================================")
        rospy.loginfo(" POINT TRAJECTORY NODE")
        rospy.loginfo("------------------------------------------")
        rospy.loginfo(" Takeoff height : %.2f m", self.takeoff_height)
        rospy.loginfo(" Max speed      : %.2f m/s", self.max_speed)
        rospy.loginfo(" Max acc        : %.2f m/s^2", self.max_acc)
        rospy.loginfo("")
        rospy.loginfo(" RViz goal:")
        rospy.loginfo("   /contest2/move_base_simple/goal")
        rospy.loginfo("")
        rospy.loginfo(" Terminal goal:")
        rospy.loginfo("   /contest2/ego_uav_interface/goal")
        rospy.loginfo("")
        rospy.loginfo(" Mission requires fresh RC mission permission")
        rospy.loginfo(" Mission requires ARMED + OFFBOARD")
        rospy.loginfo(" Takeoff target is fixed relative to HOME z")
        rospy.loginfo(" DOES NOT ARM")
        rospy.loginfo(" DOES NOT CHANGE PX4 MODE")
        rospy.loginfo("==========================================")

    # ========================================================
    # callbacks
    # ========================================================

    def odom_callback(self, msg):

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        values = [
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w
        ]

        if not all(math.isfinite(v) for v in values):

            rospy.logerr_throttle(
                0.5,
                "POINT_ODOM_FINITE_GUARD_V1: "
                "rejecting non-finite MAVROS odom"
            )
            return

        self.odom = msg
        self.odom_time = rospy.Time.now()

        if not self.initialized and self.health_ok():

            self.capture_current_hold()
            self.home_p = list(self.hold_p)

            self.initialized = True
            self.mode = "HOLD_GROUND"

            rospy.logwarn(
                "Initial HOLD captured: "
                "[%.3f %.3f %.3f], yaw=%.1f deg",
                self.hold_p[0],
                self.hold_p[1],
                self.hold_p[2],
                math.degrees(self.hold_yaw)
            )

    def health_callback(self, msg):

        self.flight_healthy = msg.data
        self.health_time = rospy.Time.now()

    def mission_permission_callback(self, msg):

        self.mission_permission = msg.data
        self.mission_permission_time = rospy.Time.now()

    def state_callback(self, msg):

        self.px4_state = msg
        self.state_time = rospy.Time.now()


    def range_callback(self, msg):

        if math.isfinite(msg.range) and msg.range > 0.0:
            self.range_value = msg.range
            self.range_time = rospy.Time.now()

    # ========================================================
    # health
    # ========================================================

    def odom_fresh(self):

        return (
            self.odom_time is not None and
            (rospy.Time.now() - self.odom_time).to_sec()
            < self.odom_timeout
        )


    def range_fresh(self):

        return (
            self.range_time is not None
            and
            self.range_value is not None
            and
            (rospy.Time.now() - self.range_time).to_sec()
            < self.range_timeout
        )

    def health_ok(self):

        return (
            self.health_time is not None and
            self.flight_healthy and
            (rospy.Time.now() - self.health_time).to_sec()
            < self.health_timeout
        )

    def final_drop_grace_active(self):

        return (
            self.traj_active
            and self.traj_kind == "LAND"
            and self.land_terminal_active
            and self.land_terminal_start_time is not None
            and (
                rospy.Time.now()
                - self.land_terminal_start_time
            ).to_sec() <= self.land_terminal_fault_grace
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

    def state_fresh(self):

        return (
            self.state_time is not None and
            (
                rospy.Time.now() - self.state_time
            ).to_sec() < self.state_timeout
        )

    def px4_execution_ok(self):

        return (
            self.state_fresh() and
            self.px4_state is not None and
            self.px4_state.connected and
            self.px4_state.armed and
            self.px4_state.mode == self.offboard_mode
        )

    # ========================================================
    # utilities
    # ========================================================

    def current_position(self):

        return [
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y,
            self.odom.pose.pose.position.z
        ]

    def current_yaw(self):

        q = self.odom.pose.pose.orientation

        _, _, yaw = euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])

        return yaw

    def capture_current_hold(self):

        self.hold_p = self.current_position()
        self.hold_yaw = self.current_yaw()

    @staticmethod
    def distance(a, b):

        return math.sqrt(
            (a[0] - b[0]) ** 2 +
            (a[1] - b[1]) ** 2 +
            (a[2] - b[2]) ** 2
        )

    def is_airborne_by_height(self):

        if self.odom is None or not self.initialized:
            return False

        return (
            self.odom.pose.pose.position.z
            >
            self.home_p[2] + self.airborne_height_threshold
        )

    def ensure_trajectory_interface_enabled(self):

        try:

            response = self.enable_trajectory_client()

            if not response.success:

                rospy.logerr(
                    "Trajectory interface enable rejected: %s",
                    response.message
                )

                return False

            return True

        except rospy.ServiceException as exc:

            rospy.logerr(
                "Cannot enable trajectory interface: %s",
                str(exc)
            )

            return False

    def abort_to_current_hold(self, reason):

        old_kind = self.traj_kind

        self.traj_active = False
        self.traj_kind = ""
        self.traj_t0 = None

        if self.odom is not None:

            self.capture_current_hold()

            airborne = self.is_airborne_by_height()

            # If the vehicle is essentially back at the ground
            # reference, clear the completed-takeoff state.
            if not airborne:
                self.takeoff_completed = False
                self.mode = "HOLD_GROUND"
            else:
                self.mode = "AIRBORNE_HOLD"

        else:

            self.mode = "WAIT_ODOM"

        rospy.logwarn(
            "%s aborted: %s -> %s",
            old_kind if old_kind else "Trajectory",
            reason,
            self.mode
        )

    def compute_duration(self, p0, pf, vmax, amax, minimum):

        d = self.distance(p0, pf)

        if d < 1e-3:
            return minimum

        # Quintic s(t)=10r^3-15r^4+6r^5
        #
        # max(ds/dr)     = 1.875
        # max(|d2s/dr2|) ≈ 5.7735

        Tv = 1.875 * d / max(vmax, 0.01)

        Ta = math.sqrt(
            5.7735 * d / max(amax, 0.01)
        )

        return max(minimum, Tv, Ta)

    # ========================================================
    # trajectory
    # ========================================================

    def begin_trajectory(
        self,
        pf,
        kind,
        vmax,
        amax,
        minimum
    ):

        self.p0 = self.current_position()
        self.pf = list(pf)

        self.traj_T = self.compute_duration(
            self.p0,
            self.pf,
            vmax,
            amax,
            minimum
        )

        self.traj_t0 = rospy.Time.now()
        self.traj_active = True
        self.traj_kind = kind

        self.hold_yaw = self.current_yaw()

        rospy.logwarn(
            "%s started: "
            "[%.2f %.2f %.2f] -> "
            "[%.2f %.2f %.2f], T=%.2f s",
            kind,
            self.p0[0],
            self.p0[1],
            self.p0[2],
            self.pf[0],
            self.pf[1],
            self.pf[2],
            self.traj_T
        )

    def evaluate_trajectory(self):

        now_ros = rospy.Time.now()

        # ====================================================
        # UAV2 LAND TERMINAL DESCENT
        #
        # The normal UAV1 quintic profile is retained above the
        # ground-effect region. Once TFMini reports a low height,
        # hold XY and yaw while maintaining a nonzero sink rate.
        # ====================================================

        if self.traj_kind == "LAND":

            if (
                not self.land_terminal_active
                and self.range_fresh()
                and self.range_value
                    <= self.land_terminal_range
            ):

                p_now = self.current_position()

                self.land_terminal_active = True

                self.land_terminal_xy = [
                    p_now[0],
                    p_now[1]
                ]

                self.land_terminal_z_ref = p_now[2]
                self.land_terminal_last_time = now_ros
                self.land_terminal_start_time = now_ros

                self.mode = "LAND_TERMINAL_DESCENT"

                rospy.logwarn(
                    "LAND TERMINAL DESCENT: "
                    "range=%.3f m, "
                    "vz=-%.3f m/s",
                    self.range_value,
                    self.land_terminal_speed
                )

            if self.land_terminal_active:

                dt = (
                    now_ros
                    - self.land_terminal_last_time
                ).to_sec()

                dt = max(
                    0.0,
                    min(dt, 0.05)
                )

                self.land_terminal_last_time = now_ros

                z_min = (
                    self.home_p[2]
                    +
                    self.land_terminal_min_offset
                )

                next_z = max(
                    z_min,
                    self.land_terminal_z_ref
                    -
                    self.land_terminal_speed * dt
                )

                still_descending = (
                    next_z > z_min + 1e-4
                )

                self.land_terminal_z_ref = next_z

                p = [
                    self.land_terminal_xy[0],
                    self.land_terminal_xy[1],
                    self.land_terminal_z_ref
                ]

                v = [
                    0.0,
                    0.0,
                    -self.land_terminal_speed
                    if still_descending
                    else 0.0
                ]

                a = [
                    0.0,
                    0.0,
                    0.0
                ]

                return p, v, a

        elapsed = (
            now_ros - self.traj_t0
        ).to_sec()

        r = elapsed / self.traj_T

        if r >= 1.0:

            self.hold_p = list(self.pf)

            finished_kind = self.traj_kind

            self.traj_active = False
            self.traj_kind = ""

            if finished_kind == "TAKEOFF":

                self.takeoff_completed = True
                self.mode = "AIRBORNE_HOLD"

                rospy.logwarn(
                    "TAKEOFF trajectory completed -> AIRBORNE_HOLD"
                )

            elif finished_kind == "LAND":

                self.takeoff_completed = False
                self.mode = "LANDED_HOLD"

                rospy.logwarn(
                    "LAND trajectory completed -> LANDED_HOLD"
                )

            else:

                self.mode = "AIRBORNE_HOLD"

                rospy.logwarn(
                    "POINT trajectory completed -> AIRBORNE_HOLD"
                )

            return (
                list(self.hold_p),
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0]
            )

        r = max(0.0, min(1.0, r))

        r2 = r * r
        r3 = r2 * r
        r4 = r3 * r
        r5 = r4 * r

        s = (
            10.0 * r3
            - 15.0 * r4
            + 6.0 * r5
        )

        ds_dr = (
            30.0 * r2
            - 60.0 * r3
            + 30.0 * r4
        )

        d2s_dr2 = (
            60.0 * r
            - 180.0 * r2
            + 120.0 * r3
        )

        ds_dt = ds_dr / self.traj_T
        d2s_dt2 = d2s_dr2 / (self.traj_T ** 2)

        dp = [
            self.pf[i] - self.p0[i]
            for i in range(3)
        ]

        p = [
            self.p0[i] + dp[i] * s
            for i in range(3)
        ]

        v = [
            dp[i] * ds_dt
            for i in range(3)
        ]

        a = [
            dp[i] * d2s_dt2
            for i in range(3)
        ]

        return p, v, a

    # ========================================================
    # services
    # ========================================================

    def start_takeoff(self, req):

        if not self.initialized:
            return TriggerResponse(
                False,
                "Not initialized"
            )

        if not self.mission_permission_fresh():
            return TriggerResponse(
                False,
                "Rejected: RC mission permission unavailable/stale"
            )

        if not self.mission_permission:
            return TriggerResponse(
                False,
                "Rejected: RC mission permission is false"
            )

        if not self.state_fresh():
            return TriggerResponse(
                False,
                "Rejected: PX4 state unavailable/stale"
            )

        if not self.px4_execution_ok():
            return TriggerResponse(
                False,
                "Rejected: PX4 must be connected, ARMED and OFFBOARD"
            )

        if not self.health_ok() or not self.odom_fresh():
            return TriggerResponse(
                False,
                "Flight health / odom not ready"
            )

        if self.traj_active:
            return TriggerResponse(
                False,
                "Trajectory already active"
            )

        if self.takeoff_completed:
            return TriggerResponse(
                False,
                "Takeoff already completed"
            )

        p = self.current_position()

        # IMPORTANT:
        # Always use a fixed HOME-relative takeoff altitude.
        # If takeoff was interrupted halfway, a new explicit
        # start_takeoff command only flies the REMAINING height.
        target_z = (
            self.home_p[2] + self.takeoff_height
        )

        # If another mechanism has already placed the vehicle
        # essentially at the mission altitude, synchronize the
        # internal state instead of commanding another climb.
        if p[2] >= target_z - 0.05:

            self.capture_current_hold()
            self.takeoff_completed = True
            self.mode = "AIRBORNE_HOLD"

            return TriggerResponse(
                True,
                "Already at takeoff altitude; state synchronized"
            )

        if not self.ensure_trajectory_interface_enabled():
            return TriggerResponse(
                False,
                "Could not enable trajectory interface"
            )

        pf = [
            p[0],
            p[1],
            target_z
        ]

        self.mode = "TAKEOFF_TRAJECTORY"

        self.begin_trajectory(
            pf,
            "TAKEOFF",
            self.takeoff_speed,
            self.takeoff_acc,
            3.0
        )

        return TriggerResponse(
            True,
            "Takeoff trajectory started"
        )

    def start_land(self, req):

        if not self.initialized:
            return TriggerResponse(
                False,
                "Not initialized"
            )

        if not self.mission_permission_fresh():
            return TriggerResponse(
                False,
                "Rejected: RC mission permission unavailable/stale"
            )

        if not self.mission_permission:
            return TriggerResponse(
                False,
                "Rejected: RC mission permission is false"
            )

        if not self.state_fresh():
            return TriggerResponse(
                False,
                "Rejected: PX4 state unavailable/stale"
            )

        if not self.px4_execution_ok():
            return TriggerResponse(
                False,
                "Rejected: PX4 must be connected, ARMED and OFFBOARD"
            )

        if not self.health_ok() or not self.odom_fresh():
            return TriggerResponse(
                False,
                "Flight health / odom not ready"
            )

        if self.traj_active:
            return TriggerResponse(
                False,
                "Trajectory already active"
            )

        # Also allow a software landing after an interrupted
        # takeoff if the measured vehicle is already airborne.
        if (
            not self.takeoff_completed
            and not self.is_airborne_by_height()
        ):
            return TriggerResponse(
                False,
                "Vehicle is not in an airborne mission state"
            )

        if not self.ensure_trajectory_interface_enabled():
            return TriggerResponse(
                False,
                "Could not enable trajectory interface"
            )

        p = self.current_position()

        # Command slightly below the captured ground level.
        # This prevents the vehicle from hovering in ground effect
        # a few centimeters above the floor.
        pf = [
            p[0],
            p[1],
            self.home_p[2] - 0.10
        ]

        self.mode = "LAND_TRAJECTORY"

        # RESET UAV2 LAND TERMINAL STATE
        self.land_terminal_active = False
        self.land_terminal_last_time = None
        self.land_terminal_start_time = None

        self.begin_trajectory(
            pf,
            "LAND",
            self.land_speed,
            self.land_acc,
            3.0
        )

        return TriggerResponse(
            True,
            "Landing trajectory started"
        )

    def abort_trajectory(self, req):

        if not self.odom_fresh():
            return TriggerResponse(
                False,
                "No fresh odom"
            )

        self.abort_to_current_hold(
            "manual/service abort"
        )

        return TriggerResponse(
            True,
            "Trajectory aborted; holding current position"
        )

    # ========================================================
    # goal
    # ========================================================

    def goal_callback(self, msg):

        if not self.mission_permission_fresh():

            rospy.logwarn(
                "Goal rejected: RC mission permission unavailable/stale"
            )
            return

        if not self.mission_permission:

            rospy.logwarn(
                "Goal rejected: RC mission permission is false"
            )
            return

        if not self.px4_execution_ok():

            rospy.logwarn(
                "Goal rejected: PX4 must be ARMED and OFFBOARD"
            )
            return

        if not self.takeoff_completed:

            rospy.logwarn(
                "Goal rejected: takeoff not completed"
            )
            return

        if self.traj_active:

            rospy.logwarn(
                "Goal rejected: trajectory already active"
            )
            return

        if not self.health_ok() or not self.odom_fresh():

            rospy.logwarn(
                "Goal rejected: health/odom invalid"
            )
            return

        p = self.current_position()

        # x/y from goal.
        # Altitude remains at the current mission HOLD altitude.
        pf = [
            msg.pose.position.x,
            msg.pose.position.y,
            self.hold_p[2]
        ]

        dxy = math.hypot(
            pf[0] - p[0],
            pf[1] - p[1]
        )

        if dxy > 3.0:

            rospy.logerr(
                "Goal rejected: %.2f m away (>3.0 m safety limit)",
                dxy
            )
            return

        if not self.ensure_trajectory_interface_enabled():

            rospy.logerr(
                "Goal rejected: could not enable trajectory interface"
            )
            return

        self.mode = "POINT_TRAJECTORY"

        self.begin_trajectory(
            pf,
            "POINT",
            self.max_speed,
            self.max_acc,
            self.min_duration
        )

    # ========================================================
    # output
    # ========================================================

    def publish_command(self, p, v, a):

        msg = PositionTarget()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # p + v + a + yaw + yaw_rate
        msg.type_mask = 0

        msg.position.x = p[0]
        msg.position.y = p[1]
        msg.position.z = p[2]

        msg.velocity.x = v[0]
        msg.velocity.y = v[1]
        msg.velocity.z = v[2]

        msg.acceleration_or_force.x = a[0]
        msg.acceleration_or_force.y = a[1]
        msg.acceleration_or_force.z = a[2]

        msg.yaw = self.hold_yaw
        msg.yaw_rate = 0.0

        self.cmd_pub.publish(msg)

    def timer_callback(self, event):

        if not self.initialized:

            self.status_pub.publish(
                String(data="WAIT_INITIALIZATION")
            )
            return

        if not self.odom_fresh():

            if self.final_drop_grace_active():

                rospy.logwarn_throttle(
                    0.20,
                    "FINAL_DROP_10CM_V2: "
                    "odom stale; continuing locked touchdown command"
                )

            else:

                # Never resume an old time-parameterized trajectory
                # after odometry has disappeared.
                if self.traj_active:

                    self.traj_active = False
                    self.traj_kind = ""
                    self.traj_t0 = None
                    self.mode = "WAIT_ODOM"

                    rospy.logerr(
                        "Active trajectory cancelled: odom lost/stale"
                    )

                self.status_pub.publish(
                    String(data="WAIT_ODOM")
                )
                return

        if not self.health_ok():

            if self.final_drop_grace_active():

                rospy.logwarn_throttle(
                    0.20,
                    "FINAL_DROP_10CM_V2: "
                    "flight health lost; continuing locked touchdown command"
                )

            else:

                if self.traj_active:
                    self.abort_to_current_hold(
                        "flight health lost"
                    )

                self.status_pub.publish(
                    String(data="WAIT_FLIGHT_HEALTHY")
                )
                return

        # ----------------------------------------------------
        # Hard mission-execution gates
        # ----------------------------------------------------

        if self.traj_active and not self.mission_permission_ok():

            self.abort_to_current_hold(
                "RC mission permission lost/stale"
            )

        if self.traj_active and not self.px4_execution_ok():

            self.abort_to_current_hold(
                "PX4 no longer ARMED/OFFBOARD"
            )

        # ----------------------------------------------------
        # Trajectory / hold command
        # ----------------------------------------------------

        if self.traj_active:

            p, v, a = self.evaluate_trajectory()

        else:

            p = list(self.hold_p)
            v = [0.0, 0.0, 0.0]
            a = [0.0, 0.0, 0.0]

        self.publish_command(p, v, a)

        self.status_pub.publish(
            String(data=self.mode)
        )


if __name__ == "__main__":

    rospy.init_node("point_traj_node")

    PointTrajectoryNode()

    rospy.spin()
