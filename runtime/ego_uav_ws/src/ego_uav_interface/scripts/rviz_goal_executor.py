#!/usr/bin/env python3

import math
import threading
import time

import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, PositionTarget
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from traj_utils.msg import Bspline
from quadrotor_msgs.msg import PositionCommand


EXPECTED_MASK = 2048

# Planning fence
X_MIN = -2.8
X_MAX =  3.3
Y_MIN = -1.5
Y_MAX =  1.5

# First-command protection
MAX_FIRST_DXY = 0.25
MAX_FIRST_DZ  = 0.15

# Completion
ARRIVE_XY = 0.15
ARRIVE_Z = 0.10
ARRIVE_SPEED = 0.20
ARRIVE_STABLE_TIME = 0.60

# Planner may occasionally need some time
PLAN_TIMEOUT = 12.0
CMD_TIMEOUT = 2.0

# Mission watchdog
# World-frame z has an arbitrary mocap origin.
# Do NOT use an absolute z threshold to determine airborne state.
# Height protection is relative to the altitude at mission start.
MAX_ALTITUDE_DROP = 0.20

# Deliberately generous.
# A real fault is detected by the other checks first.
MISSION_TIMEOUT = 60.0

POSITION_CMD_STALE = 0.60


class RvizGoalExecutor:

    def __init__(self):

        self.lock = threading.Lock()
        self.busy = False

        self.odom = None
        self.state = None
        self.mission_allowed = False

        self.mux_status = ""

        self.bspline_id = None
        self.bspline_rx_count = 0

        self.waiting_bspline = False
        self.bspline_baseline_count = 0
        self.bspline_baseline_id = None

        self.new_bspline_id = None
        self.bspline_event = threading.Event()

        self.first_cmd = None
        self.first_cmd_event = threading.Event()

        self.last_position_cmd_time = None

        # ----------------------------------------------------
        # Publishers
        # ----------------------------------------------------

        self.ego_goal_pub = rospy.Publisher(
            "/contest2/move_base_simple/goal",
            PoseStamped,
            queue_size=1
        )

        # Used by visualizer + console.
        self.active_goal_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/active_rviz_goal",
            PoseStamped,
            queue_size=1,
            latch=True
        )

        self.status_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/rviz_goal_status",
            String,
            queue_size=1,
            latch=True
        )

        # ----------------------------------------------------
        # Subscribers
        # ----------------------------------------------------

        rospy.Subscriber(
            "/contest2/ego_uav_interface/rviz_goal",
            PoseStamped,
            self.goal_cb,
            queue_size=1
        )

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
            "/contest2/ego_uav_interface/rc_mission_allowed",
            Bool,
            self.mission_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/trajectory_mux_status",
            String,
            self.mux_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/drone_0_planning/bspline",
            Bspline,
            self.bspline_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/position_cmd",
            PositionCommand,
            self.position_cmd_cb,
            queue_size=100
        )

        self.status_pub.publish(
            String(data="READY")
        )

        rospy.loginfo("======================================")
        rospy.loginfo(" RVIZ POSITION-ONLY EGO EXECUTOR READY")
        rospy.loginfo(" P + V + A")
        rospy.loginfo(" yaw / yaw_rate ignored")
        rospy.loginfo(" expected mask = 3072")
        rospy.loginfo("======================================")


    # ========================================================
    # Callbacks
    # ========================================================

    def odom_cb(self, msg):
        self.odom = msg


    def state_cb(self, msg):
        self.state = msg


    def mission_cb(self, msg):
        self.mission_allowed = msg.data


    def mux_cb(self, msg):
        self.mux_status = msg.data


    def bspline_cb(self, msg):

        self.bspline_rx_count += 1
        self.bspline_id = msg.traj_id

        if not self.waiting_bspline:
            return

        # Must be a B-spline received AFTER the RViz goal.
        if self.bspline_rx_count <= self.bspline_baseline_count:
            return

        # Prefer a genuinely new trajectory ID.
        if (
            self.bspline_baseline_id is not None and
            msg.traj_id == self.bspline_baseline_id
        ):
            return

        self.new_bspline_id = msg.traj_id
        self.waiting_bspline = False

        self.bspline_event.set()


    def position_cmd_cb(self, msg):

        self.last_position_cmd_time = rospy.Time.now()

        if self.new_bspline_id is None:
            return

        if msg.trajectory_id != self.new_bspline_id:
            return

        if self.first_cmd is None:

            self.first_cmd = msg
            self.first_cmd_event.set()


    # ========================================================
    # Services
    # ========================================================

    def call_trigger(self, name):

        rospy.wait_for_service(
            name,
            timeout=2.0
        )

        return rospy.ServiceProxy(
            name,
            Trigger
        )()


    def safe_hold(self):

        try:
            self.call_trigger(
                "/contest2/ego_uav_interface/disable_trajectory"
            )
        except Exception:
            pass

        try:
            self.call_trigger(
                "/contest2/ego_uav_interface/use_point"
            )
        except Exception:
            pass


    def abort(self, reason):

        rospy.logerr(
            "[RVIZ EGO] ABORT: %s",
            reason
        )

        self.status_pub.publish(
            String(data="ABORT: " + reason)
        )

        self.safe_hold()


    # ========================================================
    # New RViz goal
    # ========================================================

    def goal_cb(self, msg):

        with self.lock:

            if self.busy:

                rospy.logwarn(
                    "[RVIZ EGO] busy - new goal ignored"
                )
                return

            self.busy = True

        t = threading.Thread(
            target=self.execute,
            args=(msg,)
        )

        t.daemon = True
        t.start()


    def execute(self, msg):

        try:
            self.execute_impl(msg)

        except Exception as e:
            self.abort(str(e))

        finally:

            self.waiting_bspline = False

            with self.lock:
                self.busy = False

            # Explicit handshake for the mission FSM.
            # A new goal may only be sent after this event.
            self.status_pub.publish(
                String(data="IDLE")
            )


    # ========================================================
    # Mission
    # ========================================================

    def execute_impl(self, msg):

        # ----------------------------------------------------
        # Initial state
        # ----------------------------------------------------

        if self.odom is None:
            raise RuntimeError("odometry unavailable")

        if self.state is None:
            raise RuntimeError("PX4 state unavailable")

        if not self.state.armed:
            raise RuntimeError("vehicle not ARMED")

        if self.state.mode != "OFFBOARD":
            raise RuntimeError(
                "PX4 is not OFFBOARD"
            )

        if not self.mission_allowed:
            raise RuntimeError(
                "mission permission FALSE"
            )

        p0 = self.odom.pose.pose.position

        sx = p0.x
        sy = p0.y
        sz = p0.z


        gx = msg.pose.position.x
        gy = msg.pose.position.y

        # We intentionally keep current altitude.
        gz = sz

        if not (
            X_MIN <= gx <= X_MAX and
            Y_MIN <= gy <= Y_MAX
        ):
            raise RuntimeError(
                "goal outside EGO safe area"
            )

        mission_distance = math.hypot(
            gx - sx,
            gy - sy
        )

        # ----------------------------------------------------
        # Put aircraft into safe point HOLD first
        # ----------------------------------------------------

        self.status_pub.publish(
            String(data="PREPARING")
        )

        self.safe_hold()
        rospy.sleep(0.15)

        # ====================================================
        # PLAN_ALIGN_EXECUTE_V4
        #
        # P1:
        #   plan -> trigger -> validate -> execute
        #
        # P2:
        #   plan once
        #   -> preview tangent
        #   -> rotate in place
        #   -> trigger SAME trajectory
        #   -> validate
        #   -> release TANGENT translation
        # ====================================================

        p2_cfg_x = rospy.get_param(
            "/contest2/auto_course_mission/p2_x",
            -2.2
        )

        p2_cfg_y = rospy.get_param(
            "/contest2/auto_course_mission/p2_y",
            0.5
        )

        align_before_start = (
            math.hypot(
                gx - p2_cfg_x,
                gy - p2_cfg_y
            )
            < 0.08
        )

        yaw_mode_pub = None
        yaw_target_pub = None


        if align_before_start:

            from std_msgs.msg import Float64

            if self.odom is None:
                raise RuntimeError(
                    "no odometry before P2 preview"
                )

            q = (
                self.odom.pose.pose.orientation
            )

            current_yaw = math.atan2(
                2.0 * (
                    q.w * q.z
                    + q.x * q.y
                ),
                1.0 - 2.0 * (
                    q.y * q.y
                    + q.z * q.z
                )
            )


            yaw_mode_pub = rospy.Publisher(
                "/contest2/ego_uav_interface/yaw_mode",
                String,
                queue_size=1,
                latch=True
            )

            yaw_target_pub = rospy.Publisher(
                "/contest2/ego_uav_interface/yaw_target_deg",
                Float64,
                queue_size=1,
                latch=True
            )


            # Preserve current heading first.
            yaw_target_pub.publish(
                Float64(
                    data=math.degrees(
                        current_yaw
                    )
                )
            )

            yaw_mode_pub.publish(
                String(data="FIXED")
            )


            # Allow bridge to capture the old final P1
            # PositionCommand before the P2 B-spline arrives.
            rospy.sleep(0.30)


            rospy.logwarn(
                "[RVIZ EGO] P2 PREVIEW: "
                "XYZ locked at P1, yaw=%.1f deg",
                math.degrees(
                    current_yaw
                )
            )

        # ----------------------------------------------------
        # Publish the accepted goal for RViz + console
        # ----------------------------------------------------

        accepted = PoseStamped()

        accepted.header.stamp = rospy.Time.now()
        accepted.header.frame_id = "world"

        accepted.pose.position.x = gx
        accepted.pose.position.y = gy
        accepted.pose.position.z = gz

        # Keep original RViz orientation only for display.
        # It is NOT sent to PX4.
        accepted.pose.orientation = (
            msg.pose.orientation
        )

        self.active_goal_pub.publish(
            accepted
        )

        # ----------------------------------------------------
        # Prepare NEW B-spline detector BEFORE publishing goal
        # ----------------------------------------------------

        self.bspline_baseline_count = (
            self.bspline_rx_count
        )

        self.bspline_baseline_id = (
            self.bspline_id
        )

        self.new_bspline_id = None
        self.first_cmd = None

        self.bspline_event.clear()
        self.first_cmd_event.clear()

        self.waiting_bspline = True

        # ----------------------------------------------------
        # Send planner goal
        # ----------------------------------------------------

        ego_goal = PoseStamped()

        ego_goal.header.stamp = rospy.Time.now()
        ego_goal.header.frame_id = "world"

        ego_goal.pose.position.x = gx
        ego_goal.pose.position.y = gy
        ego_goal.pose.position.z = gz

        ego_goal.pose.orientation.w = 1.0

        deadline = (
            rospy.Time.now()
            + rospy.Duration(2.0)
        )

        while (
            self.ego_goal_pub.get_num_connections() == 0 and
            rospy.Time.now() < deadline
        ):
            rospy.sleep(0.02)

        if self.ego_goal_pub.get_num_connections() == 0:
            raise RuntimeError(
                "EGO planner goal subscriber missing"
            )

        self.status_pub.publish(
            String(data="PLANNING")
        )

        rospy.logwarn(
            "[RVIZ EGO] goal=(%.3f %.3f %.3f)",
            gx, gy, gz
        )

        self.ego_goal_pub.publish(
            ego_goal
        )

        # ----------------------------------------------------
        # Wait for actual new trajectory
        # ----------------------------------------------------

        if not self.bspline_event.wait(
            PLAN_TIMEOUT
        ):
            raise RuntimeError(
                "no NEW EGO B-spline within %.0fs"
                % PLAN_TIMEOUT
            )

        traj_id = self.new_bspline_id

        rospy.logwarn(
            "[RVIZ EGO] NEW B-spline id=%d",
            traj_id
        )

        # ----------------------------------------------------
        # Wait until traj_server is actually publishing the
        # newly previewed trajectory.
        # ----------------------------------------------------

        from quadrotor_msgs.msg import PositionCommand

        def wait_matching_cmd(
            max_wait,
            newer_than=None
        ):

            deadline = (
                time.time()
                + max_wait
            )

            while (
                time.time() < deadline
                and not rospy.is_shutdown()
            ):

                try:

                    candidate = rospy.wait_for_message(
                        "/contest2/position_cmd",
                        PositionCommand,
                        timeout=0.20
                    )

                except rospy.ROSException:
                    continue


                if (
                    int(candidate.trajectory_id)
                    != int(traj_id)
                ):
                    continue


                if (
                    newer_than is not None
                    and
                    candidate.header.stamp
                    <= newer_than
                ):
                    continue


                return candidate


            return None


        preview_cmd = wait_matching_cmd(
            CMD_TIMEOUT
        )

        if preview_cmd is None:
            raise RuntimeError(
                "no PositionCommand for preview trajectory"
            )


        # ----------------------------------------------------
        # P2 only:
        # obtain the INITIAL stable trajectory tangent while
        # EGO's planner FSM is frozen.
        # ----------------------------------------------------

        tangent_yaw = None

        if align_before_start:

            tangent_samples = []

            tangent_deadline = (
                time.time() + 3.0
            )

            while (
                time.time()
                < tangent_deadline
                and not rospy.is_shutdown()
            ):

                try:

                    pcmd = rospy.wait_for_message(
                        "/contest2/position_cmd",
                        PositionCommand,
                        timeout=0.20
                    )

                except rospy.ROSException:
                    continue


                if (
                    int(pcmd.trajectory_id)
                    != int(traj_id)
                ):
                    raise RuntimeError(
                        "P2 preview trajectory_id changed "
                        "%d -> %d"
                        %
                        (
                            traj_id,
                            int(pcmd.trajectory_id)
                        )
                    )


                vx = pcmd.velocity.x
                vy = pcmd.velocity.y

                vxy = math.hypot(
                    vx,
                    vy
                )

                if vxy < 0.06:
                    continue


                psi = math.atan2(
                    vy,
                    vx
                )

                tangent_samples.append(
                    psi
                )

                if (
                    len(tangent_samples) > 8
                ):
                    tangent_samples.pop(0)


                if (
                    len(tangent_samples) < 8
                ):
                    continue


                cx = sum(
                    math.cos(a)
                    for a in tangent_samples
                )

                cy = sum(
                    math.sin(a)
                    for a in tangent_samples
                )

                mean_yaw = math.atan2(
                    cy,
                    cx
                )

                spread = max(
                    abs(
                        math.atan2(
                            math.sin(
                                a - mean_yaw
                            ),
                            math.cos(
                                a - mean_yaw
                            )
                        )
                    )
                    for a in tangent_samples
                )


                if (
                    spread
                    <= math.radians(15.0)
                ):

                    tangent_yaw = (
                        mean_yaw
                    )

                    rospy.logwarn(
                        "[RVIZ EGO] "
                        "P2 START TANGENT "
                        "%.1f deg, spread=%.1f deg, "
                        "traj_id=%d",
                        math.degrees(
                            tangent_yaw
                        ),
                        math.degrees(
                            spread
                        ),
                        traj_id
                    )

                    break


            if tangent_yaw is None:
                raise RuntimeError(
                    "P2 initial tangent unavailable"
                )


        # ----------------------------------------------------
        # Verify bridge contract BEFORE selecting it.
        # ----------------------------------------------------

        bridge = rospy.wait_for_message(
            "/contest2/ego_uav_interface/ego_setpoint",
            PositionTarget,
            timeout=1.0
        )

        if bridge.type_mask != EXPECTED_MASK:
            raise RuntimeError(
                "bridge mask=%d expected=%d"
                %
                (
                    bridge.type_mask,
                    EXPECTED_MASK
                )
            )


        # ----------------------------------------------------
        # Select EGO while trajectory_interface is still
        # DISABLED by safe_hold().
        #
        # Therefore changing mux alone cannot move the vehicle.
        # ----------------------------------------------------

        r = self.call_trigger(
            "/contest2/ego_uav_interface/use_ego"
        )

        if not r.success:
            raise RuntimeError(
                "use_ego rejected: "
                + r.message
            )


        mux_deadline = (
            time.time() + 1.5
        )

        while (
            time.time() < mux_deadline
        ):

            if (
                self.mux_status
                == "EGO_ACTIVE"
            ):
                break

            rospy.sleep(0.02)


        if (
            self.mux_status
            != "EGO_ACTIVE"
        ):
            raise RuntimeError(
                "mux did not reach EGO_ACTIVE"
            )


        # ----------------------------------------------------
        # P2:
        # enable while bridge is FIXED.
        #
        # Translation is still locked; only yaw can rotate.
        # ----------------------------------------------------

        if align_before_start:

            r = self.call_trigger(
                "/contest2/ego_uav_interface/enable_trajectory"
            )

            if not r.success:
                raise RuntimeError(
                    "enable_trajectory rejected "
                    "during P2 alignment: "
                    + r.message
                )


            yaw_target_pub.publish(
                Float64(
                    data=math.degrees(
                        tangent_yaw
                    )
                )
            )

            yaw_mode_pub.publish(
                String(data="FIXED")
            )


            rospy.logwarn(
                "[RVIZ EGO] ALIGN P2: "
                "rotate in place -> %.1f deg",
                math.degrees(
                    tangent_yaw
                )
            )


            yaw_tol = math.radians(
                rospy.get_param(
                    "/contest2/auto_course_mission/"
                    "yaw_tolerance_deg",
                    8.0
                )
            )

            stable_required = rospy.get_param(
                "/contest2/auto_course_mission/"
                "yaw_stable_time",
                0.8
            )

            yaw_timeout = rospy.get_param(
                "/contest2/auto_course_mission/"
                "yaw_timeout",
                15.0
            )


            yaw_deadline = (
                time.time()
                + yaw_timeout
            )

            stable_since = None
            last_print = 0.0


            while not rospy.is_shutdown():

                if self.odom is None:
                    raise RuntimeError(
                        "odometry lost "
                        "during P2 yaw alignment"
                    )

                if (
                    self.state is None
                    or not self.state.armed
                ):
                    raise RuntimeError(
                        "vehicle disarmed "
                        "during P2 yaw alignment"
                    )

                if (
                    self.state.mode
                    != "OFFBOARD"
                ):
                    raise RuntimeError(
                        "PX4 left OFFBOARD "
                        "during P2 yaw alignment"
                    )


                q = (
                    self.odom.pose.pose.orientation
                )

                actual_yaw = math.atan2(
                    2.0 * (
                        q.w * q.z
                        + q.x * q.y
                    ),
                    1.0 - 2.0 * (
                        q.y * q.y
                        + q.z * q.z
                    )
                )

                err = math.atan2(
                    math.sin(
                        tangent_yaw
                        - actual_yaw
                    ),
                    math.cos(
                        tangent_yaw
                        - actual_yaw
                    )
                )

                now_wall = time.time()


                if (
                    now_wall - last_print
                    >= 0.25
                ):

                    print(
                        "\r\033[2K"
                        "P2 ALIGN  actual=%7.1f  "
                        "target=%7.1f  "
                        "err=%5.1f deg"
                        %
                        (
                            math.degrees(
                                actual_yaw
                            ),
                            math.degrees(
                                tangent_yaw
                            ),
                            math.degrees(
                                abs(err)
                            )
                        ),
                        end="",
                        flush=True
                    )

                    last_print = (
                        now_wall
                    )


                if abs(err) <= yaw_tol:

                    if stable_since is None:
                        stable_since = now_wall

                    elif (
                        now_wall - stable_since
                        >= stable_required
                    ):

                        print()

                        rospy.logwarn(
                            "[RVIZ EGO] "
                            "P2 ALIGN OK "
                            "actual=%.1f deg",
                            math.degrees(
                                actual_yaw
                            )
                        )

                        break

                else:

                    stable_since = None


                if (
                    now_wall
                    > yaw_deadline
                ):

                    print()

                    raise RuntimeError(
                        "P2 yaw alignment timeout"
                    )


                rospy.sleep(0.02)


        # ----------------------------------------------------
        # RELEASE SAME PREVIEW TRAJECTORY
        #
        # EGO + traj_server both subscribe to this topic and
        # both reset the SAME trajectory clock using this
        # message timestamp.
        # ----------------------------------------------------

        trigger_pub = rospy.Publisher(
            "/contest2/traj_start_trigger",
            PoseStamped,
            queue_size=1
        )


        connection_deadline = (
            time.time() + 2.0
        )

        while (
            trigger_pub.get_num_connections() < 2
            and
            time.time() < connection_deadline
            and
            not rospy.is_shutdown()
        ):
            rospy.sleep(0.02)


        if (
            trigger_pub.get_num_connections()
            < 2
        ):
            raise RuntimeError(
                "/contest2/traj_start_trigger requires "
                "both EGO and traj_server subscribers"
            )


        trigger_msg = PoseStamped()

        trigger_msg.header.stamp = (
            rospy.Time.now()
        )

        trigger_msg.header.frame_id = (
            "world"
        )

        trigger_msg.pose.position.x = sx
        trigger_msg.pose.position.y = sy
        trigger_msg.pose.position.z = sz
        trigger_msg.pose.orientation.w = 1.0


        trigger_stamp = (
            trigger_msg.header.stamp
        )


        rospy.logwarn(
            "[RVIZ EGO] START SAME TRAJECTORY "
            "id=%d from t=0",
            traj_id
        )

        trigger_pub.publish(
            trigger_msg
        )


        # ----------------------------------------------------
        # POST-trigger first-gap validation.
        #
        # Ignore all old preview commands and wait for the
        # reset trajectory to appear close to the REAL vehicle.
        # ----------------------------------------------------

        reset_deadline = (
            time.time() + 1.0
        )

        post_cmd = None

        last_dxy = float("inf")
        last_dz = float("inf")


        while (
            time.time() < reset_deadline
            and not rospy.is_shutdown()
        ):

            candidate = wait_matching_cmd(
                0.25,
                newer_than=trigger_stamp
            )

            if candidate is None:
                continue


            last_dxy = math.hypot(
                candidate.position.x - sx,
                candidate.position.y - sy
            )

            last_dz = abs(
                candidate.position.z - sz
            )


            if (
                last_dxy <= MAX_FIRST_DXY
                and
                last_dz <= MAX_FIRST_DZ
            ):

                post_cmd = candidate
                break


        rospy.logwarn(
            "[RVIZ EGO] "
            "post-trigger first gap "
            "dXY=%.3f dZ=%.3f",
            last_dxy,
            last_dz
        )


        if post_cmd is None:
            raise RuntimeError(
                "same trajectory did not restart "
                "safely from current vehicle state"
            )


        # ----------------------------------------------------
        # Release physical translation.
        # ----------------------------------------------------

        if align_before_start:

            # It was already enabled for yaw-only FIXED mode.
            # Switching to TANGENT releases P/V/A execution.
            yaw_mode_pub.publish(
                String(data="TANGENT")
            )

            rospy.sleep(0.05)

            rospy.logwarn(
                "[RVIZ EGO] "
                "P2 TANGENT execution released."
            )

        else:

            # P1 remained in trajectory_interface HOLD until
            # post-trigger first-gap was proven safe.
            r = self.call_trigger(
                "/contest2/ego_uav_interface/enable_trajectory"
            )

            if not r.success:
                raise RuntimeError(
                    "enable_trajectory rejected: "
                    + r.message
                )

        # THIS is the authoritative "mission started" event
        # used by ego_console.py.
        self.status_pub.publish(
            String(
                data="TRANSLATE traj_id=%d"
                % traj_id
            )
        )

        rospy.logwarn(
            "[RVIZ EGO] TRAJECTORY ACTIVE id=%d",
            traj_id
        )

        # ----------------------------------------------------
        # Mission monitor
        # ----------------------------------------------------

        start_time = time.time()
        stable_since = None

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self.odom is None:
                raise RuntimeError(
                    "odometry lost"
                )

            if self.state is None:
                raise RuntimeError(
                    "PX4 state lost"
                )

            if not self.state.armed:
                raise RuntimeError(
                    "vehicle DISARMED"
                )

            if self.state.mode != "OFFBOARD":
                raise RuntimeError(
                    "PX4 left OFFBOARD"
                )

            if self.mux_status != "EGO_ACTIVE":
                raise RuntimeError(
                    "EGO trajectory source not active"
                )

            now = rospy.Time.now()

            if self.last_position_cmd_time is None:
                raise RuntimeError(
                    "PositionCommand unavailable"
                )

            cmd_age = (
                now -
                self.last_position_cmd_time
            ).to_sec()

            if cmd_age > POSITION_CMD_STALE:
                raise RuntimeError(
                    "PositionCommand stale %.2fs"
                    % cmd_age
                )

            p = self.odom.pose.pose.position


            if p.z < sz - MAX_ALTITUDE_DROP:
                raise RuntimeError(
                    "excessive altitude drop"
                )

            xy_error = math.hypot(
                p.x - gx,
                p.y - gy
            )

            z_error = abs(
                p.z - gz
            )

            v = self.odom.twist.twist.linear

            speed = math.sqrt(
                v.x * v.x +
                v.y * v.y +
                v.z * v.z
            )

            self.status_pub.publish(
                String(
                    data=(
                        "RUNNING "
                        "traj_id=%d "
                        "dxy=%.3f "
                        "dz=%.3f "
                        "v=%.3f"
                        %
                        (
                            traj_id,
                            xy_error,
                            z_error,
                            speed
                        )
                    )
                )
            )

            arrived = (
                xy_error < ARRIVE_XY and
                z_error < ARRIVE_Z and
                speed < ARRIVE_SPEED
            )

            if arrived:

                if stable_since is None:
                    stable_since = time.time()

                elif (
                    time.time() -
                    stable_since
                    >= ARRIVE_STABLE_TIME
                ):
                    break

            else:
                stable_since = None

            if (
                time.time() - start_time
                > MISSION_TIMEOUT
            ):
                raise RuntimeError(
                    "mission timeout after %.0fs"
                    % MISSION_TIMEOUT
                )

            rate.sleep()

        # ----------------------------------------------------
        # COMPLETED
        #
        # Do NOT switch back to POINT here.
        # traj_server continuously holds the final point.
        # ----------------------------------------------------

        p = self.odom.pose.pose.position

        elapsed = (
            time.time() -
            start_time
        )

        self.status_pub.publish(
            String(
                data=(
                    "DONE "
                    "traj_id=%d "
                    "x=%.3f y=%.3f z=%.3f "
                    "time=%.1f"
                    %
                    (
                        traj_id,
                        p.x, p.y, p.z,
                        elapsed
                    )
                )
            )
        )

        rospy.logwarn(
            "======================================"
        )

        rospy.logwarn(
            " TARGET REACHED"
        )

        rospy.logwarn(
            " traj_id=%d",
            traj_id
        )

        rospy.logwarn(
            " final=(%.3f %.3f %.3f)",
            p.x, p.y, p.z
        )

        rospy.logwarn(
            " time=%.1fs",
            elapsed
        )

        rospy.logwarn(
            "======================================"
        )


if __name__ == "__main__":

    rospy.init_node(
        "rviz_goal_executor"
    )

    RvizGoalExecutor()

    rospy.spin()
