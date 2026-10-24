#!/usr/bin/env python3

import math
import os
import time
import threading
import subprocess
import yaml
from pathlib import Path
from demo5_config import load_config, demo_to_world
from demo8_terminal_yaw import demo_yaw_to_world_deg, TerminalYawGate

import rospy
from geometry_msgs.msg import Point
import rosgraph

from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State, PositionTarget
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import Bool, String, Float64
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from traj_utils.msg import Bspline
from quadrotor_msgs.msg import PositionCommand


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# DEMO9_EGO_START_GAP_REPLAN_V1: retry only this exact continuity fault.
class Demo9EgoStartGap(RuntimeError):
    pass


class TangentPreview:

    def __init__(self):

        self.root = Path(__file__).resolve().parent
        self.qr_proc = None

        self.session_path = (
            self.root
            / "session_frame.yaml"
        )

        self.odom = None
        self.state = None

        self.point_status = ""
        self.point_status_rx_time = None
        self.point_result = ""
        self.point_result_rx_time = None
        self.point_result_seq = 0
        self.mux_status = ""
        self.mux_status_time = None
        self.demo_safe = False

        # DIRECT_TANGENT_EXEC_V1
        self.flight_healthy = False
        self.mission_allowed = False
        # Arrival times for the recovery-only live safety gate.
        self.state_rx_time = None
        self.health_rx_time = None
        self.permission_rx_time = None

        self.cmd = None
        self.cmd_time = None
        # Demo8: monitor bridge output, do not publish PositionTarget ourselves.
        self.ego_setpoint = None
        self.ego_setpoint_time = None
        self.bridge_status = ''
        self.bridge_status_time = None
        self.if_status = ''
        self.if_status_time = None
        self.mavros_sp = None
        self.mavros_sp_time = None
        self.odom_time = None

        # RVIZ_REPLAN_TRAJ_V1
        self.traj_marker_pub = rospy.Publisher(
            "/contest1/uav2/replan_traj",
            Marker,
            queue_size=5,
            latch=True
        )

        # REALTIME_TRAJ_RVIZ_V1
        # Continuous commanded trajectory. This naturally contains
        # the path actually followed across EGO replans.
        self.executed_traj_pub = rospy.Publisher(
            "/contest1/uav2/executed_traj",
            Marker,
            queue_size=1,
            latch=True
        )

        self.executed_traj_points = []

        # ACTUAL_TRAJ_RVIZ_V1
        # Real aircraft trajectory from PX4 odometry.
        self.actual_traj_pub = rospy.Publisher(
            "/contest1/uav2/actual_traj",
            Marker,
            queue_size=1,
            latch=True
        )

        self.actual_traj_points = []

        self.bspline = None
        self.bspline_count = 0
        self.bspline_event = threading.Event()

        self.goal_pub = rospy.Publisher(
            "/contest1/move_base_simple/goal",
            PoseStamped,
            queue_size=1
        )
        # POINT-specific goal topic is intentionally isolated from EGO/RViz.
        self.point_goal_pub = rospy.Publisher(
            "/contest1/ego_uav_interface/point_goal",
            PoseStamped,
            queue_size=1
        )
        self.point_cmd = None
        self.point_cmd_time = None
        rospy.Subscriber(
            "/contest1/ego_uav_interface/point_setpoint",
            PositionTarget,
            self.point_cmd_cb,
            queue_size=20
        )

        self.active_wp_pub = rospy.Publisher(
            "/contest1/demo5/active_waypoint", String,
            queue_size=1, latch=True
        )

        self.path_pub = rospy.Publisher(
            "/contest1/uav2_static/ego_path",
            RosPath,
            queue_size=1,
            latch=True
        )

        self.tangent_pub = rospy.Publisher(
            "/contest1/yaw_tangent_demo/tangent_markers",
            MarkerArray,
            queue_size=1,
            latch=True
        )

        # DIRECT_TANGENT_EXEC_V1
        self.yaw_mode_pub = rospy.Publisher(
            "/contest1/ego_uav_interface/yaw_mode",
            String,
            queue_size=1,
            latch=True
        )
        self.yaw_target_pub = rospy.Publisher(
            "/contest1/ego_uav_interface/yaw_target_deg",
            Float64,
            queue_size=1,
            latch=False
        )
        rospy.Subscriber(
            "/contest1/ego_uav_interface/ego_setpoint",
            PositionTarget, self.ego_setpoint_cb, queue_size=20
        )
        rospy.Subscriber(
            "/contest1/ego_uav_interface/ego_bridge_status",
            String, self.bridge_status_cb, queue_size=20
        )
        rospy.Subscriber(
            "/contest1/ego_uav_interface/trajectory_interface_status",
            String, self.if_status_cb, queue_size=20
        )
        rospy.Subscriber(
            "/contest1/mavros/setpoint_raw/local",
            PositionTarget, self.mavros_sp_cb, queue_size=20
        )

        self.trigger_pub = rospy.Publisher(
            "/contest1/traj_start_trigger",
            PoseStamped,
            queue_size=1
        )

        rospy.Subscriber(
            "/contest1/mavros/local_position/odom",
            Odometry,
            self.odom_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest1/mavros/state",
            State,
            self.state_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest1/ego_uav_interface/point_traj_status",
            String,
            self.point_cb,
            queue_size=10
        )
        rospy.Subscriber(
            "/contest1/ego_uav_interface/point_traj_result",
            String, self.point_result_cb, queue_size=10
        )

        rospy.Subscriber(
            "/contest1/ego_uav_interface/trajectory_mux_status",
            String,
            self.mux_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest1/ego_uav_interface/demo_frame_safe",
            Bool,
            self.safe_cb,
            queue_size=10
        )

        # DIRECT_TANGENT_EXEC_V1
        rospy.Subscriber(
            "/contest1/ego_uav_interface/flight_healthy",
            Bool,
            self.health_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest1/ego_uav_interface/rc_mission_allowed",
            Bool,
            self.mission_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest1/position_cmd",
            PositionCommand,
            self.cmd_cb,
            queue_size=100
        )

        # ACTUAL_TRAJ_RVIZ_V1
        rospy.Subscriber(
            "/contest1/mavros/local_position/odom",
            Odometry,
            self.actual_odom_cb,
            queue_size=50
        )

        rospy.Subscriber(
            "/contest1/drone_0_planning/bspline",
            Bspline,
            self.bspline_cb,
            queue_size=10
        )

        rospy.loginfo(
            "Demo5 waypoint EGO runner ready. "
            "PLAN -> VALIDATE -> MOVE + TANGENT."
        )


    def odom_cb(self, msg):
        self.odom = msg
        self.odom_time = rospy.Time.now()


    def ego_setpoint_cb(self, msg):
        self.ego_setpoint = msg
        self.ego_setpoint_time = rospy.Time.now()

    def bridge_status_cb(self, msg):
        self.bridge_status = msg.data
        self.bridge_status_time = rospy.Time.now()

    def if_status_cb(self, msg):
        self.if_status = msg.data
        self.if_status_time = rospy.Time.now()

    def mavros_sp_cb(self, msg):
        self.mavros_sp = msg
        self.mavros_sp_time = rospy.Time.now()

    def state_cb(self, msg):
        self.state = msg
        self.state_rx_time = rospy.Time.now()


    def point_cb(self, msg):
        self.point_status = msg.data
        self.point_status_rx_time = rospy.Time.now()

    def point_result_cb(self, msg):
        self.point_result = msg.data
        self.point_result_rx_time = rospy.Time.now()
        self.point_result_seq += 1

    def point_cmd_cb(self, msg):
        self.point_cmd = msg
        self.point_cmd_time = rospy.Time.now()

    def mux_cb(self, msg):
        self.mux_status = msg.data
        self.mux_status_time = rospy.Time.now()


    def safe_cb(self, msg):
        self.demo_safe = msg.data


    # DIRECT_TANGENT_EXEC_V1
    def health_cb(self, msg):
        self.flight_healthy = msg.data
        self.health_rx_time = rospy.Time.now()


    def mission_cb(self, msg):
        self.mission_allowed = msg.data
        self.permission_rx_time = rospy.Time.now()



    # ACTUAL_TRAJ_RVIZ_V1
    def actual_odom_cb(self, msg):

        p = msg.pose.pose.position

        q = Point()
        q.x = p.x
        q.y = p.y
        q.z = p.z

        append = False

        if not self.actual_traj_points:
            append = True

        else:
            last = self.actual_traj_points[-1]

            d = (
                (q.x-last.x)**2
                +
                (q.y-last.y)**2
                +
                (q.z-last.z)**2
            ) ** 0.5

            if d >= 0.02:
                append = True


        if append:

            self.actual_traj_points.append(q)

            if len(self.actual_traj_points) > 3000:
                self.actual_traj_points = \
                    self.actual_traj_points[-3000:]


            marker = Marker()

            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()

            marker.ns = "actual_trajectory"
            marker.id = 0

            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.03

            # Blue
            marker.color.r = 0.0
            marker.color.g = 0.4
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker.points = list(
                self.actual_traj_points
            )

            self.actual_traj_pub.publish(marker)


    def cmd_cb(self, msg):
        self.cmd = msg
        self.cmd_time = rospy.Time.now()

        # REALTIME_TRAJ_RVIZ_V1
        q = Point()
        q.x = msg.position.x
        q.y = msg.position.y
        q.z = msg.position.z

        append_point = False

        if not self.executed_traj_points:
            append_point = True
        else:
            p0 = self.executed_traj_points[-1]

            d = math.sqrt(
                (q.x - p0.x) ** 2
                +
                (q.y - p0.y) ** 2
                +
                (q.z - p0.z) ** 2
            )

            # About 2 cm spatial resolution.
            if d >= 0.02:
                append_point = True

        if append_point:

            self.executed_traj_points.append(q)

            # Safety cap; far above what this 5-m demo needs.
            if len(self.executed_traj_points) > 2000:
                self.executed_traj_points =                     self.executed_traj_points[-2000:]

            marker = Marker()

            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()

            marker.ns = "executed_trajectory"
            marker.id = 0

            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.025

            # Orange: actually commanded/executed trajectory.
            marker.color.r = 1.0
            marker.color.g = 0.55
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.points = list(
                self.executed_traj_points
            )

            self.executed_traj_pub.publish(marker)


    def bspline_cb(self, msg):

        self.bspline = msg
        self.bspline_count += 1

        self.bspline_event.set()


    def wait_traj_server_ack(
        self,
        traj_id,
        timeout_s=4.0
    ):

        deadline = time.time() + timeout_s

        first_xyz = None
        first_time = None

        max_move = 0.0
        matched = 0

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
        ):

            try:
                msg = rospy.wait_for_message(
                    "/contest1/position_cmd",
                    PositionCommand,
                    timeout=0.30
                )

            except rospy.ROSException:
                continue

            if int(msg.trajectory_id) != int(traj_id):
                continue

            xyz = (
                msg.position.x,
                msg.position.y,
                msg.position.z
            )

            if first_xyz is None:

                first_xyz = xyz
                first_time = time.time()

            move = math.sqrt(
                (xyz[0] - first_xyz[0]) ** 2
                +
                (xyz[1] - first_xyz[1]) ** 2
                +
                (xyz[2] - first_xyz[2]) ** 2
            )

            max_move = max(
                max_move,
                move
            )

            matched += 1

            # Require several commands over a short interval,
            # not just one accidental matching message.
            if (
                matched >= 5
                and
                time.time() - first_time >= 0.10
            ):

                if max_move >= 0.20:
                    raise RuntimeError(
                        "traj_server ACK received but "
                        "preview is not frozen: "
                        "traj_id=%d max_cmd_move=%.4f m"
                        %
                        (
                            traj_id,
                            max_move
                        )
                    )

                rospy.logwarn(
                    "[ OK ] TRAJ_SERVER ACK: "
                    "traj_id=%d, preview frozen, "
                    "max_cmd_move=%.4f m",
                    traj_id,
                    max_move
                )

                return


        raise RuntimeError(
            "traj_server did not ACK B-spline "
            "traj_id=%d on /contest1/position_cmd"
            % traj_id
        )


    def current_yaw(self):

        q = self.odom.pose.pose.orientation

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


    # ============================================================
    # B-spline
    # ============================================================

    @staticmethod
    def de_boor(cps, knots, p, u):

        n = len(cps) - 1

        if u >= knots[n + 1]:
            k = n
        else:
            k = p

            for i in range(p, n + 1):

                if knots[i] <= u < knots[i + 1]:
                    k = i
                    break

        d = [
            list(cps[j])
            for j in range(k - p, k + 1)
        ]

        for r in range(1, p + 1):

            for j in range(p, r - 1, -1):

                idx = k - p + j

                den = (
                    knots[idx + p - r + 1]
                    -
                    knots[idx]
                )

                if abs(den) < 1.0e-12:
                    a = 0.0
                else:
                    a = (
                        u - knots[idx]
                    ) / den

                for q in range(3):

                    d[j][q] = (
                        (1.0 - a)
                        * d[j - 1][q]
                        +
                        a
                        * d[j][q]
                    )

        return d[p]


    def sample_bspline(self, msg):

        cps = [
            (
                p.x,
                p.y,
                p.z
            )
            for p in msg.pos_pts
        ]

        knots = list(msg.knots)
        degree = int(msg.order)

        n = len(cps) - 1

        if n < degree:
            raise RuntimeError(
                "invalid B-spline control points"
            )

        if len(knots) < n + degree + 2:
            raise RuntimeError(
                "invalid knot vector"
            )

        u0 = knots[degree]
        u1 = knots[n + 1]

        if u1 <= u0:
            raise RuntimeError(
                "invalid B-spline domain"
            )

        out = []

        N = 301

        for i in range(N):

            u = (
                u0
                +
                (u1 - u0)
                * i
                / float(N - 1)
            )

            out.append(
                self.de_boor(
                    cps,
                    knots,
                    degree,
                    u
                )
            )

        return out, u0, u1


    # ============================================================
    # Demo coordinate transform
    # ============================================================

    def load_session(self):

        if not self.session_path.exists():
            raise RuntimeError(
                "session_frame.yaml missing"
            )

        with open(
            self.session_path,
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

        fence = self.session["demo_fence"]

        self.x_min = float(
            fence["forward_min"]
        )
        self.x_max = float(
            fence["forward_max"]
        )

        self.y_min = float(
            fence["left_min"]
        )
        self.y_max = float(
            fence["left_max"]
        )

        self.h_min = float(
            fence["height_min"]
        )
        self.h_max = float(
            fence["height_max"]
        )


    def to_demo(self, p):

        dx = p[0] - self.ox
        dy = p[1] - self.oy

        xd = (
            self.fx * dx
            +
            self.fy * dy
        )

        yd = (
            self.lx * dx
            +
            self.ly * dy
        )

        hd = p[2] - self.oz

        return (
            xd,
            yd,
            hd
        )


    # ============================================================
    # Visualization
    # ============================================================

    def publish_path(self, pts):

        msg = RosPath()

        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()

        for p in pts:

            ps = PoseStamped()

            ps.header = msg.header

            ps.pose.position.x = p[0]
            ps.pose.position.y = p[1]
            ps.pose.position.z = p[2]

            ps.pose.orientation.w = 1.0

            msg.poses.append(ps)

        self.path_pub.publish(msg)


    def publish_tangents(self, pts):

        arr = MarkerArray()

        now = rospy.Time.now()

        marker_id = 0

        # Avoid zero-speed B-spline endpoints.
        indices = list(
            range(
                12,
                len(pts) - 12,
                18
            )
        )

        for i in indices:

            p0 = pts[i - 3]
            p1 = pts[i + 3]

            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]

            n = math.hypot(
                dx,
                dy
            )

            if n < 0.01:
                continue

            ux = dx / n
            uy = dy / n

            m = Marker()

            m.header.frame_id = "world"
            m.header.stamp = now

            m.ns = "tangent_yaw"
            m.id = marker_id

            marker_id += 1

            m.type = Marker.ARROW
            m.action = Marker.ADD

            a = Point()
            b = Point()

            a.x = pts[i][0]
            a.y = pts[i][1]
            a.z = pts[i][2] + 0.08

            b.x = a.x + 0.28 * ux
            b.y = a.y + 0.28 * uy
            b.z = a.z

            m.points = [
                a,
                b
            ]

            m.scale.x = 0.025
            m.scale.y = 0.070
            m.scale.z = 0.070

            # Tangent arrows: magenta
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.color.a = 1.0

            arr.markers.append(m)

        self.tangent_pub.publish(arr)


    # ============================================================
    # Analysis
    # ============================================================

    @staticmethod
    def tangent_yaw_near_start(pts):

        p0 = pts[0]

        for i in range(
            1,
            min(
                40,
                len(pts)
            )
        ):

            dx = pts[i][0] - p0[0]
            dy = pts[i][1] - p0[1]

            if math.hypot(
                dx,
                dy
            ) > 0.03:

                return math.atan2(
                    dy,
                    dx
                )

        return None


    @staticmethod
    def tangent_yaw_near_end(pts):

        p1 = pts[-1]

        for i in range(
            len(pts) - 2,
            max(
                0,
                len(pts) - 50
            ),
            -1
        ):

            dx = p1[0] - pts[i][0]
            dy = p1[1] - pts[i][1]

            if math.hypot(
                dx,
                dy
            ) > 0.03:

                return math.atan2(
                    dy,
                    dx
                )

        return None


    def execution_initial_yaw(self, msg):

        """
        Robust pre-alignment direction.

        Do NOT lower traj_server's 0.10 m validity threshold.

        For low-speed trajectories, the nominal 1-s lookahead can
        contain less than 0.10 m of displacement.  For ALIGN only,
        search farther along the SAME B-spline until a meaningful
        horizontal displacement is available.

        At real execution start, traj_server will later be synchronized
        to the already-aligned actual vehicle yaw.
        """

        cps = [
            (
                p.x,
                p.y,
                p.z
            )
            for p in msg.pos_pts
        ]

        knots = list(msg.knots)
        degree = int(msg.order)

        n = len(cps) - 1

        u0 = knots[degree]
        u1 = knots[n + 1]

        nominal_tf = float(
            rospy.get_param(
                "/contest1/drone_0_traj_server/"
                "traj_server/time_forward",
                1.0
            )
        )

        # Alignment needs a geometrically meaningful direction.
        min_align_dist = 0.15

        p0 = self.de_boor(
            cps,
            knots,
            degree,
            u0
        )

        duration = u1 - u0

        # Start with the real traj_server lookahead, then extend.
        lookahead = max(
            0.10,
            nominal_tf
        )

        best_dist = 0.0
        best_lookahead = lookahead

        while lookahead <= duration + 1.0e-9:

            u = min(
                u0 + lookahead,
                u1
            )

            pf = self.de_boor(
                cps,
                knots,
                degree,
                u
            )

            dx = pf[0] - p0[0]
            dy = pf[1] - p0[1]

            dist = math.hypot(
                dx,
                dy
            )

            best_dist = dist
            best_lookahead = lookahead

            if dist >= min_align_dist:

                yaw = math.atan2(
                    dy,
                    dx
                )

                return (
                    yaw,
                    nominal_tf,
                    dist,
                    lookahead
                )

            if u >= u1:
                break

            lookahead += 0.20

        return (
            None,
            nominal_tf,
            best_dist,
            best_lookahead
        )


    def validated_ego_path(self, msg):
        pts, _, _ = self.sample_bspline(msg)
        demos = [self.to_demo(p) for p in pts]
        h_target = float(self.session["goal_demo"]["height"])
        for f, l, h in demos:
            if not all(math.isfinite(v) for v in (f,l,h)):
                raise RuntimeError("non-finite EGO path or replan")
            if not (self.x_min <= f <= self.x_max and
                    self.y_min <= l <= self.y_max and
                    self.h_min <= h <= self.h_max):
                raise RuntimeError("EGO path/replan outside Demo5 fence")
            # No duplicate target-height band check here.
            # Retain finite-value and physical flight-fence checks above.
        return pts

    def analyze(
        self,
        msg,
        pts,
        traj_id,
        u0,
        u1
    ):

        demos = [
            self.to_demo(p)
            for p in pts
        ]

        xmin = min(p[0] for p in demos)
        xmax = max(p[0] for p in demos)

        ymin = min(p[1] for p in demos)
        ymax = max(p[1] for p in demos)

        hmin = min(p[2] for p in demos)
        hmax = max(p[2] for p in demos)

        inside = all(
            self.x_min <= p[0] <= self.x_max
            and
            self.y_min <= p[1] <= self.y_max
            and
            self.h_min <= p[2] <= self.h_max
            for p in demos
        )

        goal = self.session["goal_world"]

        gx = float(goal["x"])
        gy = float(goal["y"])

        end_xy_err = math.hypot(
            pts[-1][0] - gx,
            pts[-1][1] - gy
        )

        actual_yaw = self.current_yaw()

        tan0 = self.tangent_yaw_near_start(
            pts
        )

        tan1 = self.tangent_yaw_near_end(
            pts
        )

        (
            exec_yaw,
            exec_time_forward,
            exec_forward_dist,
            exec_align_lookahead
        ) = self.execution_initial_yaw(
            msg
        )

        if exec_yaw is None:
            exec_mismatch = float("nan")
        else:
            exec_mismatch = math.degrees(
                wrap(
                    exec_yaw
                    -
                    actual_yaw
                )
            )

        if tan0 is None:
            mismatch = float("nan")
        else:
            mismatch = math.degrees(
                wrap(
                    tan0
                    -
                    actual_yaw
                )
            )

        lines = []

        lines.append(
            "========== 5M TANGENT PREVIEW =========="
        )

        lines.append(
            "traj_id              = %d"
            % traj_id
        )

        lines.append(
            "duration_like_span    = %.3f"
            % (u1 - u0)
        )

        lines.append(
            "current_yaw_deg       = %.2f"
            % math.degrees(
                actual_yaw
            )
        )

        lines.append(
            "initial_tangent_deg   = %s"
            %
            (
                "%.2f"
                % math.degrees(tan0)
                if tan0 is not None
                else "nan"
            )
        )

        lines.append(
            "execution_initial_yaw_deg = %s"
            %
            (
                "%.2f"
                % math.degrees(exec_yaw)
                if exec_yaw is not None
                else "nan"
            )
        )

        lines.append(
            "execution_yaw_error_deg   = %.2f"
            % exec_mismatch
        )

        lines.append(
            "yaw_time_forward          = %.2f s"
            % exec_time_forward
        )

        lines.append(
            "yaw_forward_distance      = %.3f m"
            % exec_forward_dist
        )

        lines.append(
            "alignment_lookahead       = %.2f s"
            % exec_align_lookahead
        )

        lines.append(
            "final_tangent_deg     = %s"
            %
            (
                "%.2f"
                % math.degrees(tan1)
                if tan1 is not None
                else "nan"
            )
        )

        lines.append(
            "initial_yaw_error_deg = %.2f"
            % mismatch
        )

        lines.append(
            "DEMO forward range    = "
            "[%+.3f, %+.3f] m"
            %
            (
                xmin,
                xmax
            )
        )

        lines.append(
            "DEMO left range       = "
            "[%+.3f, %+.3f] m"
            %
            (
                ymin,
                ymax
            )
        )

        lines.append(
            "DEMO height range     = "
            "[%+.3f, %+.3f] m"
            %
            (
                hmin,
                hmax
            )
        )

        lines.append(
            "end_xy_to_5m_goal     = %.3f m"
            % end_xy_err
        )

        lines.append(
            "PATH_INSIDE_DEMO_FENCE = %s"
            % str(inside)
        )

        if end_xy_err < 0.20:
            lines.append(
                "trajectory_scope      = FULL_GOAL"
            )
        else:
            lines.append(
                "trajectory_scope      = LOCAL_SEGMENT"
            )

        lines.append(
            "========================================="
        )

        text = "\n".join(lines)

        print()
        print(text)
        print()

        # DEMO5 HARD PRE-RELEASE GATE. Reject ANY planned sample outside
        # the rotated Demo fence, and reject non-finite samples.
        if not inside:
            raise RuntimeError(
                "B-spline route violates authoritative Demo5 fence"
            )
        if not all(
            math.isfinite(v) for p in demos for v in p
        ):
            raise RuntimeError("non-finite B-spline path")

        if exec_yaw is None:
            raise RuntimeError(
                "Execution initial yaw is undefined"
            )

        align_data = {
            "traj_id": int(traj_id),
            "execution_initial_yaw_deg": float(
                math.degrees(exec_yaw)
            ),
            "planning_actual_yaw_deg": float(
                math.degrees(actual_yaw)
            ),
            "execution_yaw_error_deg": float(
                exec_mismatch
            ),
            "yaw_time_forward": float(
                exec_time_forward
            ),
            "yaw_forward_distance": float(
                exec_forward_dist
            ),
            "alignment_lookahead": float(
                exec_align_lookahead
            ),
            "created_wall_time": float(
                time.time()
            ),
        }

        align_path = (
            self.root
            /
            "alignment_target.yaml"
        )

        with open(align_path, "w") as f:
            yaml.safe_dump(
                align_data,
                f,
                sort_keys=False
            )

        print(
            "[ OK ] ALIGN target saved:"
        )
        print(align_path)

        out = (
            self.root
            /
            "preview_summary.txt"
        )

        out.write_text(
            text + "\n"
        )


    # ============================================================
    # Main preview sequence
    # ============================================================

    def check_goal_subscribers(self):

        master = rosgraph.Master(
            rospy.get_name()
        )

        _, subs, _ = master.getSystemState()

        goal_subs = []

        for topic, nodes in subs:

            if topic == "/contest1/move_base_simple/goal":
                goal_subs = nodes
                break

        rospy.loginfo(
            "goal subscribers = %s",
            str(goal_subs)
        )

        if any(
            "point_traj" in n
            for n in goal_subs
        ):
            raise RuntimeError(
                "point_traj_node subscribes to "
                "/contest1/move_base_simple/goal -- unsafe"
            )

        if not any(
            "ego_planner" in n
            for n in goal_subs
        ):
            raise RuntimeError(
                "EGO goal subscriber missing"
            )


    def wait_airborne_hold(self, check_height=True):

        target_h = float(
            self.session["goal_demo"]["height"]
        )

        height_tol = rospy.get_param(
            "~preview_height_tolerance",
            0.20
        )

        max_vz = rospy.get_param(
            "~preview_max_vz",
            0.10
        )

        stable_time = rospy.get_param(
            "~preview_height_stable_time",
            0.40
        )

        stable_since = None

        rospy.loginfo(
            "Waiting for PHYSICAL airborne settle:"
        )

        rospy.loginfo(
            "  target Demo height = %.2f m",
            target_h
        )

        if check_height:
            rospy.loginfo(
                "  height tolerance   = +/- %.2f m",
                height_tol
            )
        else:
            rospy.logwarn(
                "[DEMO9] INTER-WAYPOINT HEIGHT GATE DISABLED; "
                "HOLD/mode/VZ stability checks retained"
            )

        rospy.loginfo(
            "  max |vz|           = %.2f m/s",
            max_vz
        )

        rospy.loginfo(
            "  stable time        = %.2f s",
            stable_time
        )

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            if (
                self.odom is None
                or
                self.state is None
            ):

                stable_since = None

                rospy.loginfo_throttle(
                    1.0,
                    "WAIT | odom/state unavailable"
                )

                rate.sleep()
                continue

            p = self.odom.pose.pose.position

            vz = (
                self.odom
                .twist
                .twist
                .linear
                .z
            )

            hd = p.z - self.oz

            base_ok = (
                self.state.armed
                and
                self.state.mode == "OFFBOARD"
                and
                self.point_status == "AIRBORNE_HOLD"
                and
                self.mux_status == "POINT_ACTIVE"
                and
                self.demo_safe
            )

            # Initial takeoff checks height.
            # Inter-waypoint POINT HOLD ignores fixed target height.
            height_ok = (
                (not check_height)
                or abs(hd - target_h) <= height_tol
            )

            vz_ok = (
                abs(vz)
                <= max_vz
            )

            all_ok = (
                base_ok
                and
                height_ok
                and
                vz_ok
            )

            if all_ok:

                if stable_since is None:
                    stable_since = rospy.Time.now()

                stable_age = (
                    rospy.Time.now()
                    -
                    stable_since
                ).to_sec()

            else:

                stable_since = None
                stable_age = 0.0

            rospy.loginfo_throttle(
                0.5,
                "WAIT | armed=%s mode=%s "
                "point=%s mux=%s safe=%s "
                "| h=%.3f target=%.3f "
                "| vz=%+.3f "
                "| stable=%.2f/%.2f",
                str(self.state.armed),
                self.state.mode,
                self.point_status,
                self.mux_status,
                str(self.demo_safe),
                hd,
                target_h,
                vz,
                stable_age,
                stable_time
            )

            if (
                all_ok
                and
                stable_age >= stable_time
            ):

                rospy.logwarn(
                    "========================================"
                )

                rospy.logwarn(
                    " PHYSICAL AIRBORNE SETTLE CONFIRMED"
                )

                rospy.logwarn(
                    " Demo height = %.3f m",
                    hd
                )

                rospy.logwarn(
                    " vz          = %+.3f m/s",
                    vz
                )

                rospy.logwarn(
                    "========================================"
                )

                # Give depth/map one more second while holding.
                rospy.sleep(1.0)

                return

            rate.sleep()


    # ============================================================
    # DIRECT_TANGENT_EXEC_V1
    #
    # No pre-ALIGN.
    #
    # POINT HOLD
    # -> validated frozen B-spline
    # -> select EGO while physical output is disabled
    # -> trigger SAME trajectory from t=0
    # -> validate first gap
    # -> enable P/V/A + TANGENT yaw together
    # ============================================================

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
                %
                (
                    name,
                    result.message
                )
            )


    def restore_point_hold(self):
        # DEMO9_HANDOFF_REVIEW_V4 - guarded, bounded EGO -> POINT rejoin.
        # Never re-enable the trajectory interface on an unverified state.
        # This is a DEV candidate, not approved for powered flight.
        rospy.logwarn('[HANDOFF_V4] Begin EGO -> POINT HOLD rejoin')
        disable = '/contest1/ego_uav_interface/disable_trajectory'
        abort = '/contest1/ego_uav_interface/abort_trajectory'
        use_point = '/contest1/ego_uav_interface/use_point'
        enable = '/contest1/ego_uav_interface/enable_trajectory'

        # Disable upstream trajectory and wait for a *fresh actual output*
        # from the downstream 50 Hz HOLD. MUX will use the SAME output as
        # its switch continuity anchor.
        disable_at = rospy.Time.now()
        self.call_service(disable)

        def safety_check():
            state = self.state
            if (state is None or not state.connected or not state.armed or
                    state.mode != 'OFFBOARD'):
                raise RuntimeError('POINT rejoin: PX4 not ARMED OFFBOARD')
            if not self.mission_allowed:
                raise RuntimeError('POINT rejoin: RC permission lost')
            if not self.demo_safe:
                raise RuntimeError('POINT rejoin: Demo-frame safety lost')
            now = rospy.Time.now()
            for name, stamp, timeout in (
                    ('PX4 state', self.state_rx_time, 2.50),
                    ('flight health', self.health_rx_time, 0.30),
                    ('RC permission', self.permission_rx_time, 1.50)):
                if stamp is None or not 0.0 <= (now - stamp).to_sec() < timeout:
                    raise RuntimeError('POINT rejoin: %s unavailable/stale' % name)
            if self.odom is None or self.odom_time is None:
                raise RuntimeError('POINT rejoin: no odometry')
            age = (now - self.odom_time).to_sec()
            if not 0.0 <= age < 0.20:
                raise RuntimeError('POINT rejoin: odometry stale')
            if self.if_status_time is None:
                raise RuntimeError('POINT rejoin: trajectory interface status missing')
            if_age = (now - self.if_status_time).to_sec()
            if not 0.0 <= if_age < 0.30:
                raise RuntimeError('POINT rejoin: trajectory interface status stale')

        def current_point_ok(reset_started):
            cmd = self.point_cmd
            stamp = self.point_cmd_time
            if (cmd is None or stamp is None or not stamp > reset_started or
                    not 0.0 <= (rospy.Time.now() - stamp).to_sec() < 0.15 or
                    self.point_status != 'AIRBORNE_HOLD' or
                    self.point_status_rx_time is None or
                    not 0.0 <= (rospy.Time.now() - self.point_status_rx_time).to_sec() < 0.30 or
                    self.if_status != 'HOLD_ACTIVE'):
                return False, None
            # A callback arriving after abort is not proof that the sample
            # was GENERATED after abort. POINT stamps its 50-Hz samples.
            generated = getattr(getattr(cmd,'header',None),'stamp',None)
            if (generated is None or generated.to_sec() <= 0.0 or
                    not generated > reset_started or
                    not 0.0 <= (rospy.Time.now() - generated).to_sec() < 0.15):
                return False, None
            # POINT AIRBORNE_HOLD must not retain trajectory feed-forward.
            ff_values = (cmd.velocity.x, cmd.velocity.y, cmd.velocity.z,
                         cmd.acceleration_or_force.x, cmd.acceleration_or_force.y,
                         cmd.acceleration_or_force.z, cmd.yaw_rate)
            if (not all(math.isfinite(float(v)) for v in ff_values) or
                    any(abs(float(v)) > 0.05 for v in ff_values[:3]) or
                    any(abs(float(v)) > 0.15 for v in ff_values[3:6]) or
                    abs(float(ff_values[6])) > 0.05):
                return False, None
            downstream, downstream_time = self.mavros_sp, self.mavros_sp_time
            if (downstream is None or downstream_time is None or
                    not downstream_time > disable_at or
                    not 0.0 <= (rospy.Time.now() - downstream_time).to_sec() < 0.15 or
                    downstream.coordinate_frame != PositionTarget.FRAME_LOCAL_NED):
                return False, None
            active_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                           PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_YAW)
            if downstream.type_mask & active_mask:
                return False, None
            if cmd.coordinate_frame != PositionTarget.FRAME_LOCAL_NED:
                return False, None
            mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                    PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_YAW)
            if cmd.type_mask & mask:
                return False, None
            p = self.odom.pose.pose.position
            v = self.odom.twist.twist.linear
            values = (p.x, p.y, p.z, cmd.position.x, cmd.position.y,
                      cmd.position.z, cmd.yaw, v.x, v.y, v.z,
                      downstream.position.x, downstream.position.y,
                      downstream.position.z, downstream.yaw)
            if not all(math.isfinite(float(x)) for x in values):
                return False, None
            dist = math.sqrt((cmd.position.x - p.x)**2 +
                             (cmd.position.y - p.y)**2 +
                             (cmd.position.z - p.z)**2)
            yaw_error = abs(wrap(cmd.yaw - self.current_yaw()))
            speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
            hold_dist = math.sqrt(
                (cmd.position.x-downstream.position.x)**2 +
                (cmd.position.y-downstream.position.y)**2 +
                (cmd.position.z-downstream.position.z)**2)
            hold_yaw = abs(wrap(cmd.yaw-downstream.yaw))
            return (dist <= 0.12 and yaw_error <= math.radians(12.0)
                    and speed <= 0.20 and hold_dist <= 0.12 and
                    hold_yaw <= math.radians(12.0)), generated

        # One bounded re-capture is allowed if POINT's first post-abort
        # samples were delayed. This does NOT retry a waypoint or ARM.
        verified = False
        for attempt in range(2):
            safety_check()
            self.call_service(abort)
            # Only samples produced after the successful abort ACK can
            # qualify; a pre-abort sample may arrive while service runs.
            reset_started = rospy.Time.now()
            deadline = time.monotonic() + 1.30
            good = 0
            last_good_stamp = None
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                safety_check()
                ok, stamp = current_point_ok(reset_started)
                if ok:
                    if stamp != last_good_stamp:
                        good += 1
                        last_good_stamp = stamp
                    if good >= 3:
                        verified = True
                        break
                else:
                    good = 0
                    last_good_stamp = None
                rospy.sleep(0.02)
            if verified:
                break
            rospy.logwarn('[HANDOFF_V4] POINT HOLD not verified; retry %d/2', attempt + 1)
        if not verified:
            raise RuntimeError('POINT rejoin veto: fresh current HOLD not verified')

        safety_check()
        switch_started = rospy.Time.now()
        self.call_service(use_point)
        deadline = time.monotonic() + 0.60
        active = False
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            safety_check()
            if (self.mux_status == 'POINT_ACTIVE' and
                    self.mux_status_time is not None and
                    self.mux_status_time > switch_started and
                    0.0 <= (rospy.Time.now() - self.mux_status_time).to_sec() < 0.20):
                active = True
                break
            rospy.sleep(0.02)
        if not active:
            raise RuntimeError('POINT rejoin veto: fresh MUX POINT_ACTIVE not confirmed')

        safety_check()
        enabled_at = rospy.Time.now()
        # Even a service timeout could occur AFTER the remote service ran.
        # Put the enable call inside the recovery guard as well.
        try:
            self.call_service(enable)
            # The ACK alone does not prove the actual MAVROS stream
            # has switched; verify before allowing the next waypoint.
            deadline = time.monotonic() + 0.60
            good = 0
            last_output = None
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                safety_check()
                if self.mux_status != 'POINT_ACTIVE':
                    raise RuntimeError('POINT rejoin: MUX lost POINT_ACTIVE')
                # Continue requiring a fresh, newly generated POINT HOLD
                # throughout the downstream verification window.
                point_generated = getattr(
                    getattr(self.point_cmd, 'header', None), 'stamp', None)
                point_current = (
                    self.point_status == 'AIRBORNE_HOLD' and
                    self.point_status_rx_time is not None and
                    0.0 <= (rospy.Time.now() - self.point_status_rx_time).to_sec() < 0.30 and
                    self.point_cmd_time is not None and
                    0.0 <= (rospy.Time.now() - self.point_cmd_time).to_sec() < 0.15 and
                    point_generated is not None and
                    point_generated > enabled_at and
                    0.0 <= (rospy.Time.now() - point_generated).to_sec() < 0.15)
                sp, stamp = self.mavros_sp, self.mavros_sp_time
                ok = (point_current and self.if_status == 'TRAJECTORY_ACTIVE' and
                      sp is not None and stamp is not None and stamp > enabled_at and
                      0.0 <= (rospy.Time.now() - stamp).to_sec() < 0.15 and
                      sp.coordinate_frame == PositionTarget.FRAME_LOCAL_NED and
                      (sp.type_mask & (PositionTarget.IGNORE_PX |
                       PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
                       PositionTarget.IGNORE_YAW)) == 0)
                if ok:
                    p = self.point_cmd.position
                    vals = (sp.position.x, sp.position.y, sp.position.z,
                            sp.yaw, p.x, p.y, p.z, self.point_cmd.yaw)
                    if all(math.isfinite(float(x)) for x in vals):
                        d = math.sqrt((sp.position.x-p.x)**2 +
                                      (sp.position.y-p.y)**2 +
                                      (sp.position.z-p.z)**2)
                        a = abs(wrap(sp.yaw-self.point_cmd.yaw))
                        ok = d <= 0.12 and a <= math.radians(12.0)
                    else:
                        ok = False
                if ok and stamp != last_output:
                    good += 1
                    last_output = stamp
                    if good >= 3:
                        rospy.logwarn('[HANDOFF_V4] POINT verified through MAVROS output')
                        return
                elif not ok:
                    good = 0
                    last_output = None
                rospy.sleep(0.02)
            raise RuntimeError('POINT rejoin: MAVROS POINT output not verified')
        except Exception:
            # If trajectory was enabled but output confirmation fails,
            # immediately request the existing measured-position HOLD.
            try:
                self.call_service(disable)
            except Exception as disable_error:
                rospy.logerr('[HANDOFF_V4] disable_trajectory failed: %s', disable_error)
            raise


    def wait_matching_cmd(
        self,
        traj_id,
        timeout_s,
        newer_than=None
    ):

        deadline = time.time() + timeout_s

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
        ):

            if (
                self.cmd is not None
                and
                self.cmd_time is not None
            ):

                age = (
                    rospy.Time.now()
                    -
                    self.cmd_time
                ).to_sec()

                same_id = (
                    int(self.cmd.trajectory_id)
                    ==
                    int(traj_id)
                )

                new_enough = (
                    newer_than is None
                    or
                    self.cmd.header.stamp
                    >
                    newer_than
                )

                if (
                    age < 0.30
                    and
                    same_id
                    and
                    new_enough
                ):
                    return self.cmd

            rospy.sleep(0.01)

        return None



    # RVIZ_REPLAN_TRAJ_V1
    def publish_traj_marker(self, pts, traj_id):

        marker = Marker()

        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "ego_replan"
        marker.id = int(traj_id)

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.03

        marker.pose.orientation.w = 1.0

        # Green: planned trajectory.
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for p in pts:

            q = Point()
            q.x = p[0]
            q.y = p[1]
            q.z = p[2]

            marker.points.append(q)

        self.traj_marker_pub.publish(marker)



    def stop_qr(self):
        proc, self.qr_proc = self.qr_proc, None
        if proc is None:
            return
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        rospy.logwarn('[QR] recognition OFF')


    # Final waypoint: return control to the physical pilot in POSCTL.
    def start_land(self):
        script = self.root / 'demo9_posctl_handoff.py'
        rospy.logwarn('[FINAL] Requesting POSCTL pilot landing handoff')
        ret = subprocess.call(['python3', '-B', '-u', str(script)])
        if ret != 0:
            raise RuntimeError('Final POSCTL handoff failed: rc=%d' % ret)
        rospy.logwarn('[FINAL] Physical DISARM observed after landing')


    # ============================================================
    # Demo5 V2: POINT waypoint leg.  Uses the EXISTING point_traj_node
    # whose quintic trajectory retains the yaw measured at leg start.
    # POINT offers NO obstacle avoidance, even though the depth camera
    # and EGO planner may be running on the same aircraft.
    # ============================================================

    def execute_point_waypoint(self, index, total, waypoint):
        self.active_wp_num = index
        self.wp_total = total
        f = waypoint["forward"]
        l = waypoint["left"]
        h = self.demo5_cfg["flight_height"]
        gx, gy, goal_z = demo_to_world(
            (self.ox, self.oy, self.oz),
            (self.fx, self.fy), (self.lx, self.ly), f, l, h
        )

        self.active_wp_pub.publish(String(
            data="P%d/%d POINT F%+.2f L%+.2f %s" % (
                index, total, f, l,
                "FINAL_LANDING_GOAL" if index == total else "CONTINUE"
            )
        ))
        rospy.logwarn(
            "[DEMO5 V2] POINT P%d/%d: F%+.2f L%+.2f; "
            "yaw held; NO obstacle avoidance",
            index, total, f, l
        )

        if (self.state is None or not self.state.connected or
                not self.state.armed or self.state.mode != "OFFBOARD"):
            raise RuntimeError("POINT requires ARMED OFFBOARD PX4")
        if not self.flight_healthy or not self.mission_allowed or not self.demo_safe:
            raise RuntimeError("POINT safety permission/health/guard failed")
        if self.mux_status != "POINT_ACTIVE":
            raise RuntimeError("POINT MUX must start in POINT_ACTIVE")
        if self.point_status != "AIRBORNE_HOLD":
            raise RuntimeError("POINT controller is not in AIRBORNE_HOLD")
        if self.odom is None:
            raise RuntimeError("POINT odometry unavailable")

        p = self.odom.pose.pose.position
        start_x, start_y, start_z = p.x, p.y, p.z
        start_f, start_l, start_h = self.to_demo(
            (start_x, start_y, start_z)
        )
        actual_distance = math.hypot(gx - start_x, gy - start_y)
        if not all(math.isfinite(v) for v in
                   (start_f, start_l, start_h, gx, gy, actual_distance)):
            raise RuntimeError("POINT contains non-finite coordinates")
        if not (self.x_min <= start_f <= self.x_max and
                self.y_min <= start_l <= self.y_max and
                self.h_min <= start_h <= self.h_max):
            raise RuntimeError("POINT actual start outside Demo frame fence")
        if actual_distance > 2.95:
            raise RuntimeError(
                "POINT actual leg %.3fm too long for >3m-rejecting "
                "controller" % actual_distance
            )
        if self.point_goal_pub.get_num_connections() < 1:
            raise RuntimeError("POINT goal subscriber is not connected")
        if (self.point_cmd is None or self.point_cmd_time is None or
                (rospy.Time.now() - self.point_cmd_time).to_sec() >= 0.25):
            raise RuntimeError("POINT setpoint stream is missing/stale")

        yaw_initial = self.current_yaw()
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.position.z = goal_z 
        msg.pose.orientation.w = 1.0
        result_start_seq = self.point_result_seq
        published = rospy.Time.now()
        rospy.logwarn(
            "[DEMO5 V2] POINT goal world=(%.3f,%.3f,%.3f) "
            "from=(%.3f,%.3f,%.3f) dXY=%.3f yaw=%.1fdeg",
            gx, gy, goal_z, start_x, start_y, start_z,
            actual_distance, math.degrees(yaw_initial)
        )
        # A single publish; fail rather than publishing again on uncertainty.
        self.point_goal_pub.publish(msg)

        # The existing POINT node has no goal-ACK service. Its explicit
        # POINT_TRAJECTORY state is the acknowledgement; no false success.
        acknowledged = False
        ack_end = time.time() + 3.0
        while not rospy.is_shutdown() and time.time() < ack_end:
            if self.point_status == "POINT_TRAJECTORY":
                acknowledged = True
                break
            if (self.state is None or not self.state.armed or
                    self.state.mode != "OFFBOARD" or
                    not self.flight_healthy or not self.mission_allowed or
                    not self.demo_safe or self.mux_status != "POINT_ACTIVE"):
                raise RuntimeError("POINT failed safety during goal acceptance")
            rospy.sleep(0.02)
        if not acknowledged:
            raise RuntimeError(
                "POINT goal not acknowledged as POINT_TRAJECTORY "
                "(possible distance/permission rejection)"
            )

        stable_since = None
        end = time.time() + self.segment_timeout_s
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.time() < end:
            if (self.state is None or not self.state.armed or
                    self.state.mode != "OFFBOARD"):
                raise RuntimeError("PX4 execution state lost in POINT")
            if not self.flight_healthy or not self.mission_allowed or not self.demo_safe:
                raise RuntimeError(
                    "POINT control permission changed: health=%s rc=%s demo_fence=%s "
                    "armed=%s mode=%s" % (
                        self.flight_healthy, self.mission_allowed, self.demo_safe,
                        getattr(self.state, 'armed', None),
                        getattr(self.state, 'mode', None)))
            if self.mux_status != "POINT_ACTIVE":
                raise RuntimeError("POINT MUX left POINT_ACTIVE")
            if self.point_status not in ("POINT_TRAJECTORY", "AIRBORNE_HOLD"):
                raise RuntimeError("POINT controller unexpected state: %s" %
                                   self.point_status)
            if (self.point_cmd is None or self.point_cmd_time is None or
                    (rospy.Time.now() - self.point_cmd_time).to_sec() >= 0.25):
                raise RuntimeError("POINT setpoint stale")
            cmd = self.point_cmd
            vals = (
                cmd.position.x, cmd.position.y, cmd.position.z,
                cmd.velocity.x, cmd.velocity.y, cmd.velocity.z,
                cmd.yaw
            )
            if not all(math.isfinite(v) for v in vals):
                raise RuntimeError("POINT setpoint contains NaN/Inf")
            yaw_cmd_error = abs(wrap(cmd.yaw - yaw_initial))
            if yaw_cmd_error > math.radians(12.0):
                raise RuntimeError(
                    "POINT commanded yaw changed %.1fdeg; expected hold" %
                    math.degrees(yaw_cmd_error)
                )
            p = self.odom.pose.pose.position
            v = self.odom.twist.twist.linear
            dxy = math.hypot(p.x - gx, p.y - gy)
            dz = abs(p.z - goal_z)
            speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            if not all(math.isfinite(q) for q in (dxy, dz, speed)):
                raise RuntimeError("POINT odometry contains NaN/Inf")
            # Final POINT leg uses the SAME physical-arrival criteria as
            # intermediate legs. A missed latched point_result=COMPLETED
            # must not create an indefinite hover despite a confirmed HOLD.
            if (index == total and self.point_result_seq > result_start_seq
                    and self.point_result == "ABORTED"):
                raise RuntimeError("Final POINT trajectory aborted")
            arrived = (
                self.point_status == "AIRBORNE_HOLD" and
                dxy < 0.25 and dz < 0.15 and speed < 0.20
            )
            if arrived:
                if stable_since is None:
                    stable_since = time.time()
            else:
                stable_since = None
            stable = (time.time() - stable_since) if stable_since else 0.0
            print(
                "\r[DEMO5 V2] POINT P%d/%d dXY=%.3f dZ=%.3f "
                "speed=%.3f stable=%.2f/0.30" %
                (index, total, dxy, dz, speed, stable),
                end="", flush=True
            )
            if stable >= 0.30:
                print()
                rospy.logwarn(
                    "[WAYPOINT] %d/%d reached: measured POINT HOLD stable for 0.30s",
                    index, total
                )
                if index == total:
                    self.start_land()
                return
            rate.sleep()
        raise RuntimeError(
            "POINT P%d timed out after %.1fs" %
            (index, self.segment_timeout_s)
        )

    # Demo8: ONLY the already-running EGO bridge may publish fixed-yaw P/V/A.
    # Keep MUX=EGO and trajectory_interface enabled until angular acceptance.
    # This function performs no direct MAVROS publication or software ARM.
    def execute_terminal_yaw(self, gx, gy, gz, world_yaw_deg, deadline):
        if self.odom is None:
            raise RuntimeError("Demo8 yaw: no odometry")
        if (self.yaw_mode_pub.get_num_connections() < 1 or
                self.yaw_target_pub.get_num_connections() < 1):
            raise RuntimeError("Demo8 yaw bridge mode/target subscribers missing")

        def fresh(stamp, limit):
            return stamp is not None and 0 <= (rospy.Time.now() - stamp).to_sec() < limit

        # Reject loss of permissions and data BEFORE requesting FIXED mode.
        if (self.state is None or not self.state.connected or
                not self.state.armed or self.state.mode != "OFFBOARD" or
                not self.flight_healthy or not self.mission_allowed or
                not self.demo_safe or self.mux_status != "EGO_ACTIVE" or
                not fresh(self.odom_time, 0.25) or
                not fresh(self.cmd_time, 0.35) or
                not fresh(self.ego_setpoint_time, 0.25) or
                not fresh(self.if_status_time, 0.40) or
                self.if_status != "TRAJECTORY_ACTIVE" or
                not fresh(self.mavros_sp_time, 0.25)):
            raise RuntimeError("Demo8 yaw: state/permission/command precheck failed")

        # Use only the existing bridge's yaw_target / FIXED control interface.
        # FIXED captures current odometry XYZ, and traj_server continues
        # publishing its final B-spline hover command at 100Hz.
        self.active_wp_pub.publish(String(
            data="P%d/%d EGO_TERMINAL_YAW %.1fdeg_WORLD" % (
                self.active_wp_num, self.wp_total, world_yaw_deg)))
        rospy.logwarn("[DEMO8] terminal yaw target WORLD=%+.1f deg", world_yaw_deg)
        rospy.set_param(
            "/contest2/ego_uav_interface/terminal_yaw_hold_z",
            float(gz)
	)
        self.yaw_target_pub.publish(Float64(data=world_yaw_deg))
        rospy.sleep(0.20)  # target callback precedes FIXED mode entry
        self.yaw_mode_pub.publish(String(data="FIXED"))

        gate = TerminalYawGate((gx, gy, gz), world_yaw_deg, stable_s=0.2)
        mode_ack_deadline = time.monotonic() + 2.0
        latest_replan = int(self.bspline.traj_id) if self.bspline is not None else None
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.time() < deadline:
            # No reduced CH7/health/geometry admission criteria in yaw phase.
            if (self.state is None or not self.state.connected or
                    not self.state.armed or self.state.mode != "OFFBOARD"):
                raise RuntimeError("Demo8 yaw: PX4 state lost")
            if not self.flight_healthy or not self.mission_allowed or not self.demo_safe:
                raise RuntimeError("Demo8 yaw: health/CH7/frame permission lost")
            if self.mux_status != "EGO_ACTIVE":
                raise RuntimeError("Demo8 yaw: MUX is not EGO_ACTIVE")
            if (not fresh(self.odom_time, 0.25) or
                    not fresh(self.cmd_time, 0.35) or
                    not fresh(self.ego_setpoint_time, 0.25) or
                    not fresh(self.if_status_time, 0.40) or
                    not fresh(self.mavros_sp_time, 0.25)):
                raise RuntimeError("Demo8 yaw: control or odometry stream stale")
            if self.if_status != "TRAJECTORY_ACTIVE":
                raise RuntimeError("Demo8 yaw: trajectory_interface is not active")

            candidate = self.bspline
            if candidate is not None and int(candidate.traj_id) != latest_replan:
                self.validated_ego_path(candidate)
                latest_replan = int(candidate.traj_id)
                rospy.logwarn("[DEMO8] yaw-phase replan %d fence checked", latest_replan)

            # Acknowledged FIXED mode, normal bridge output, zero translational
            # feedforward. No separate publisher can silently bypass MUX.
            mode_ack = (fresh(self.bridge_status_time, 1.25) and
                        self.bridge_status.startswith("mode=FIXED "))
            if not mode_ack and time.monotonic() > mode_ack_deadline:
                raise RuntimeError("Demo8 yaw: bridge did not ACK FIXED mode")
            sp = self.ego_setpoint
            v = sp.velocity
            a = sp.acceleration_or_force
            position = sp.position
            cmd_yaw = sp.yaw
            finite_output = all(math.isfinite(v0) for v0 in (
                position.x, position.y, position.z,
                v.x, v.y, v.z, a.x, a.y, a.z, cmd_yaw))
            if not finite_output:
                raise RuntimeError("Demo8 yaw: bridge setpoint NaN/Inf")
            mav_sp = self.mavros_sp
            # Verify trajectory_interface forwarded FIXED rather than HOLD.
            if mav_sp is None or not all(math.isfinite(v0) for v0 in (
                    mav_sp.position.x, mav_sp.position.y, mav_sp.position.z,
                    mav_sp.yaw)):
                raise RuntimeError("Demo8 yaw: invalid MAVROS input setpoint")
            mavros_matches = (
                mav_sp.type_mask == sp.type_mask and
                math.hypot(mav_sp.position.x - position.x,
                           mav_sp.position.y - position.y) <= 0.04 and
                abs(mav_sp.position.z - position.z) <= 0.04 and
                abs(math.atan2(math.sin(mav_sp.yaw - cmd_yaw),
                               math.cos(mav_sp.yaw - cmd_yaw))) <= 0.10
            )
            fixed_setpoint = (
                sp.type_mask == PositionTarget.IGNORE_YAW_RATE and
                max(abs(v.x), abs(v.y), abs(v.z),
                    abs(a.x), abs(a.y), abs(a.z)) <= 0.02 and
                math.hypot(position.x - gx, position.y - gy) < 0.25 and
                abs(position.z - gz) < 0.15 and
                abs(math.atan2(
                    math.sin(cmd_yaw - math.radians(world_yaw_deg)),
                    math.cos(cmd_yaw - math.radians(world_yaw_deg))
                )) <= math.radians(6.0) and mavros_matches
            )
            p = self.odom.pose.pose.position
            measured = (p.x, p.y, p.z)
            current_demo = self.to_demo(measured)
            if not all(math.isfinite(v0) for v0 in current_demo):
                raise RuntimeError("Demo8 yaw: invalid Demo-frame pose")
            if not (self.x_min <= current_demo[0] <= self.x_max and
                    self.y_min <= current_demo[1] <= self.y_max and
                    self.h_min <= current_demo[2] <= self.h_max):
                raise RuntimeError("Demo8 yaw: measured pose outside fence")
            # Large drift at any time is a hard failure, not a timeout.
            if (math.hypot(p.x - gx, p.y - gy) > 0.40 or
                    abs(p.z - gz) > 0.25):
                raise RuntimeError("Demo8 yaw: excessive endpoint drift")
            measured_vel = self.odom.twist.twist.linear
            done, values = gate.update(
                time.monotonic(), measured,
                (measured_vel.x, measured_vel.y, measured_vel.z),
                self.current_yaw(), bool(mode_ack and fixed_setpoint))
            dxy, dz, speed, yaw_error, stable = values
            rospy.loginfo_throttle(
                0.5, "[DEMO8] YAW dxy=%.3f dz=%.3f speed=%.3f "
                "err=%.1fdeg stable=%.2f/0.20 FIXED=%s",
                dxy, dz, speed, yaw_error, stable, str(bool(mode_ack)))
            if done:
                rospy.logwarn(
                    "[DEMO8] terminal yaw accepted: error=%.1f deg; "
                    "dxy=%.3f dz=%.3f speed=%.3f",
                    yaw_error, dxy, dz, speed)
                return
            rate.sleep()
        raise RuntimeError("Demo8 terminal yaw timeout (or ROS shutdown)")


    def execute_direct(
        self,
        traj_id,
        gx,
        gy,
        gz,
        land_after=False,
        terminal_yaw_world_deg=None
    ):

        print()
        print("========================================")
        print(" DIRECT TANGENT EXECUTION")
        print("========================================")
        print(" NO pre-ALIGN")
        print(" translation + yaw start together")
        print(" NO software ARM")
        print(" traj_id = %d" % traj_id)
        print("========================================")
        print()

        # ----------------------------------------------------
        # Preconditions
        # ----------------------------------------------------

        if (
            self.state is None
            or
            not self.state.armed
        ):
            raise RuntimeError(
                "vehicle is not ARMED"
            )

        if self.state.mode != "OFFBOARD":
            raise RuntimeError(
                "PX4 is not OFFBOARD"
            )

        if not self.flight_healthy:
            raise RuntimeError(
                "flight_healthy=False"
            )

        if not self.mission_allowed:
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

        cmd = self.wait_matching_cmd(
            traj_id,
            1.0
        )

        if cmd is None:
            raise RuntimeError(
                "fresh PositionCommand unavailable "
                "for traj_id=%d"
                % traj_id
            )

        p = self.odom.pose.pose.position

        gap_xy = math.hypot(
            cmd.position.x - p.x,
            cmd.position.y - p.y
        )

        gap_z = abs(
            cmd.position.z - p.z
        )

        print(
            "pre-release gap = "
            "dXY %.3f m  dZ %.3f m"
            %
            (
                gap_xy,
                gap_z
            )
        )

        # Keep the existing strict safety gate.
        if (
            gap_xy >= 0.20
            or
            gap_z >= 0.20
        ):
            raise Demo9EgoStartGap("pre-release first gap >= 0.20 m")

        # ----------------------------------------------------
        # Prepare TANGENT mode.
        #
        # MUX is still POINT here, therefore bridge cannot
        # physically rotate the aircraft in the background.
        # ----------------------------------------------------

        self.yaw_mode_pub.publish(
            String(data="TANGENT")
        )

        # Physical output HOLD first.
        self.call_service(
            "/contest1/ego_uav_interface/disable_trajectory"
        )

        # Select the already validated EGO command.
        self.call_service(
            "/contest1/ego_uav_interface/use_ego"
        )

        deadline = time.time() + 1.5

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
            and self.mux_status != "EGO_ACTIVE"
        ):
            rospy.sleep(0.02)

        if self.mux_status != "EGO_ACTIVE":
            raise RuntimeError(
                "MUX did not enter EGO_ACTIVE"
            )

        # ----------------------------------------------------
        # Trigger SAME B-spline from t=0.
        # ----------------------------------------------------

        deadline = time.time() + 2.0

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
            and self.trigger_pub.get_num_connections() < 2
        ):
            rospy.sleep(0.02)

        if self.trigger_pub.get_num_connections() < 2:
            raise RuntimeError(
                "/contest1/traj_start_trigger does not have "
                "both EGO + traj_server subscribers"
            )

        p = self.odom.pose.pose.position

        trig = PoseStamped()

        trig.header.stamp = rospy.Time.now()
        trig.header.frame_id = "world"

        trig.pose.position.x = p.x
        trig.pose.position.y = p.y
        trig.pose.position.z = p.z
        trig.pose.orientation.w = 1.0

        trigger_stamp = trig.header.stamp

        rospy.logwarn(
            "[DIRECT] TRIGGER SAME trajectory "
            "traj_id=%d",
            traj_id
        )

        # Exactly one trigger.
        self.trigger_pub.publish(trig)

        # ----------------------------------------------------
        # Validate reset command BEFORE releasing physical P/V/A.
        # ----------------------------------------------------

        post_cmd = self.wait_matching_cmd(
            traj_id,
            1.0,
            newer_than=trigger_stamp
        )

        if post_cmd is None:
            raise RuntimeError(
                "no fresh post-trigger PositionCommand"
            )

        p = self.odom.pose.pose.position

        post_gap_xy = math.hypot(
            post_cmd.position.x - p.x,
            post_cmd.position.y - p.y
        )

        post_gap_z = abs(
            post_cmd.position.z - p.z
        )

        print(
            "post-trigger gap = "
            "dXY %.3f m  dZ %.3f m"
            %
            (
                post_gap_xy,
                post_gap_z
            )
        )

        if (
            post_gap_xy >= 0.20
            or
            post_gap_z >= 0.20
        ):
            raise Demo9EgoStartGap("post-trigger first gap >= 0.20 m")

        # ----------------------------------------------------
        # RELEASE:
        #
        # position/velocity/acceleration and TANGENT yaw become
        # active at the SAME time.
        # ----------------------------------------------------

        self.call_service(
            "/contest1/ego_uav_interface/enable_trajectory"
        )

        rospy.logwarn(
            "[DIRECT] RELEASED: "
            "translation + TANGENT yaw"
        )

        # ----------------------------------------------------
        # Mission monitor
        # ----------------------------------------------------

        stable_since = None
        last_traj_id = traj_id
        last_checked_bspline_id = traj_id

        start_wall = time.time()
        deadline = start_wall + self.segment_timeout_s

        rate = rospy.Rate(20)

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
        ):

            # An EGO obstacle-avoidance replan is allowed for every leg,
            # but must remain inside the same authoritative rotated fence.
            candidate = self.bspline
            if (candidate is not None and
                    int(candidate.traj_id) != last_checked_bspline_id):
                self.validated_ego_path(candidate)
                rospy.logwarn(
                    "[DEMO5] In-flight EGO replan %d -> %d: fence OK",
                    last_checked_bspline_id, int(candidate.traj_id)
                )
                last_checked_bspline_id = int(candidate.traj_id)

            if (
                self.state is None
                or
                not self.state.armed
                or
                self.state.mode != "OFFBOARD"
            ):
                raise RuntimeError(
                    "PX4 execution state lost"
                )

            if not self.mission_allowed:
                raise RuntimeError(
                    "mission permission lost"
                )

            if self.mux_status != "EGO_ACTIVE":
                raise RuntimeError(
                    "MUX left EGO_ACTIVE"
                )

            if (
                self.cmd is None
                or
                self.cmd_time is None
            ):
                raise RuntimeError(
                    "PositionCommand unavailable"
                )

            cmd_age = (
                rospy.Time.now()
                -
                self.cmd_time
            ).to_sec()

            if cmd_age > 0.60:
                raise RuntimeError(
                    "PositionCommand stale %.2f s"
                    % cmd_age
                )

            tid = int(
                self.cmd.trajectory_id
            )

            if tid != last_traj_id:

                print()

                rospy.logwarn(
                    "[DIRECT] in-flight replan "
                    "%d -> %d",
                    last_traj_id,
                    tid
                )

                last_traj_id = tid

            p = self.odom.pose.pose.position

            v = self.odom.twist.twist.linear

            dxy = math.hypot(
                p.x - gx,
                p.y - gy
            )

            dz = abs(
                p.z - gz
            )

            speed = math.sqrt(
                v.x * v.x
                +
                v.y * v.y
                +
                v.z * v.z
            )

            # Track actual yaw error relative to current
            # trajectory tangent for flight evaluation only.
            vxy_cmd = math.hypot(
                self.cmd.velocity.x,
                self.cmd.velocity.y
            )

            if vxy_cmd >= 0.06:

                tangent = math.atan2(
                    self.cmd.velocity.y,
                    self.cmd.velocity.x
                )

                yaw_err_deg = math.degrees(
                    abs(
                        wrap(
                            tangent
                            -
                            self.current_yaw()
                        )
                    )
                )

            else:
                yaw_err_deg = float("nan")

            # Demo8 yaw handoff waits for a tighter stationary state.
            # Unconstrained legs preserve Demo7's thresholds.
            arrived = (
                dxy < (0.25 if terminal_yaw_world_deg is not None else 0.25)
                and
                dz < (0.15 if terminal_yaw_world_deg is not None else 0.15)
                and
                speed < (0.20 if terminal_yaw_world_deg is not None else 0.20)
            )

            if arrived:

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
                "\rDIRECT "
                "traj=%d "
                "dXY=%.3f "
                "dZ=%.3f "
                "v=%.3f "
                "yawErr=%5.1f deg "
                "stable=%.2f/0.30"
                %
                (
                    tid,
                    dxy,
                    dz,
                    speed,
                    yaw_err_deg,
                    stable
                ),
                end="",
                flush=True
            )

            if stable >= 0.30:

                print()

                rospy.logwarn(
                    "[DEMO8] WAYPOINT %d/%d TRANSLATION REACHED",
                    self.active_wp_num, self.wp_total
                )

                # Optional EGO-only endpoint orientation requirement.
                # Do not restore POINT before FIXED yaw completes, because
                # POINT would otherwise overwrite the bridge's yaw command.
                if terminal_yaw_world_deg is not None:
                    self.execute_terminal_yaw(
                        gx, gy, gz, terminal_yaw_world_deg, deadline)

                # Existing Demo7 STOP/HOLD chain: disable EGO, POINT abort,
                # MUX POINT, then re-enable under existing safety checks.
                self.restore_point_hold()
                self.yaw_mode_pub.publish(String(data="HOLD"))

                rospy.sleep(0.5)
                if (self.mux_status != "POINT_ACTIVE" or
                        self.point_status != "AIRBORNE_HOLD"):
                    raise RuntimeError("Demo8 POINT HOLD restoration failed")

                # ONLY LAST WAYPOINT triggers the proven landing chain.
                if land_after:
                    self.start_land()
                else:
                    rospy.logwarn(
                        "[DEMO5] P%d complete; POINT HOLD, NEXT leg uses EGO",
                        self.active_wp_num
                    )
                    if (
                        self.mux_status != "POINT_ACTIVE" or
                        self.point_status != "AIRBORNE_HOLD"
                    ):
                        raise RuntimeError(
                            "intermediate waypoint did not restore POINT HOLD"
                        )

                print()
                print("========================================")
                print(" DIRECT TANGENT EXECUTION PASSED")
                print(
                    " final dXY = %.3f m"
                    % dxy
                )

                print(
                    " final position = "
                    "(%.3f, %.3f, %.3f)"
                    %
                    (
                        p.x,
                        p.y,
                        p.z
                    )
                )

                print(
                    " goal position = "
                    "(%.3f, %.3f, %.3f)"
                    %
                    (
                        gx,
                        gy,
                        gz
                    )
                )
                print(" POINT HOLD captured at final position")
                print("========================================")
                print()

                return

            rate.sleep()

        raise RuntimeError(
            "Demo5 P%d EGO execution timed out after %.1f s" %
            (self.active_wp_num, self.segment_timeout_s)
        )


    # DEMO9_EGO_START_GAP_REPLAN_V1
    # Rejected EGO start continuity is not a reason to skip a waypoint.
    # The original EGO -> POINT recovery already happened in _once.
    # WAIT with POINT holding; replan the SAME goal using fresh odometry.
    def _demo9_wait_gap_replan_hold(self, timeout_s=6.0):
        stable_since = None
        start = time.monotonic()
        while not rospy.is_shutdown():
            now = time.monotonic()
            if now - start > timeout_s:
                raise RuntimeError(
                    '[DEMO9 GAP] POINT HOLD never stabilized; pilot takeover required')

            s = self.state
            if (s is None or not s.connected or not s.armed or
                    s.mode != 'OFFBOARD' or not self.flight_healthy or
                    not self.mission_allowed or not self.demo_safe):
                raise RuntimeError(
                    '[DEMO9 GAP] OFFBOARD/RC/health/fence invalid; do not replan')

            # Restore services can take a moment; POINT must be active and
            # verified before a new EGO preview is allowed.
            if (self.mux_status != 'POINT_ACTIVE' or
                    self.point_status != 'AIRBORNE_HOLD'):
                stable_since = None
                rospy.sleep(0.05)
                continue

            if (self.odom is None or self.point_cmd is None or
                    self.odom_time is None or self.point_cmd_time is None):
                raise RuntimeError('[DEMO9 GAP] odom/POINT output unavailable')
            stamp = rospy.Time.now()
            a = (stamp - self.odom_time).to_sec()
            b = (stamp - self.point_cmd_time).to_sec()
            if not (-0.05 <= a <= 0.25 and -0.05 <= b <= 0.25):
                raise RuntimeError('[DEMO9 GAP] odom/POINT setpoint stale')

            p = self.odom.pose.pose.position
            v = self.odom.twist.twist.linear
            hold = self.point_cmd.position
            values = (p.x, p.y, p.z, v.x, v.y, v.z,
                      hold.x, hold.y, hold.z)
            if not all(math.isfinite(float(x)) for x in values):
                raise RuntimeError('[DEMO9 GAP] non-finite pose/velocity/setpoint')

            dx, dy, dz = self.to_demo((p.x, p.y, p.z))
            if not (self.x_min <= dx <= self.x_max and
                    self.y_min <= dy <= self.y_max and
                    self.h_min <= dz <= self.h_max and
                    abs(dz-float(self.demo5_cfg['flight_height'])) <= 0.25):
                raise RuntimeError('[DEMO9 GAP] position outside EGO recovery fence')

            xyerr = math.hypot(p.x-hold.x, p.y-hold.y)
            vxy = math.hypot(v.x, v.y)
            stationary = (xyerr <= 0.06 and vxy <= 0.10 and
                          abs(v.z) <= 0.10)
            if stationary:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= 0.80:
                    rospy.logwarn(
                        '[DEMO9 GAP] POINT HOLD stable; track=%.3fm vxy=%.3fm/s; '
                        'replanning same waypoint from current odom', xyerr, vxy)
                    return
            else:
                stable_since = None
            rospy.sleep(0.05)
        raise RuntimeError('[DEMO9 GAP] ROS shutdown while holding')

    def plan_execute_waypoint(self, index, total, waypoint):
        # This is bounded re-planning, NEVER a bypass of the 8cm safety gate.
        # Only Demo9EgoStartGap is eligible; other errors are propagated.
        max_replans = 2
        for attempt in range(max_replans + 1):
            try:
                return self._demo9_plan_execute_once(index, total, waypoint)
            except Demo9EgoStartGap as exc:
                if attempt >= max_replans:
                    rospy.logerr(
                        '[DEMO9 GAP] P%d rejected after %d plans; '
                        'keep POINT HOLD and request pilot takeover',
                        index, max_replans+1)
                    raise RuntimeError(
                        '[DEMO9 GAP] retry limit exhausted; pilot takeover required'
                    ) from exc
                rospy.logwarn(
                    '[DEMO9 GAP] P%d EGO start rejected (%s); '
                    'POINT HOLD then plan same waypoint again (%d/%d)',
                    index, str(exc), attempt+1, max_replans)
                self._demo9_wait_gap_replan_hold()
                rospy.sleep(0.15)

    def _demo9_plan_execute_once(self, index, total, waypoint):

        self.active_wp_num = index
        self.wp_total = total
        h = self.demo5_cfg["flight_height"]
        f = waypoint["forward"]
        l = waypoint["left"]
        gx, gy, absolute_z = demo_to_world(
            (self.ox, self.oy, self.oz),
            (self.fx, self.fy), (self.lx, self.ly), f, l, h
        )
        self.session["goal_demo"] = {
            "forward": f, "left": l, "height": h
        }
        self.session["goal_world"] = {
            "x": gx, "y": gy, "z": absolute_z
        }
        self.active_wp_pub.publish(String(
            data="P%d/%d F%+.2f L%+.2f %s" % (
                index, total, f, l,
                "FINAL_LANDING_GOAL" if index == total else "EGO_CONTINUE"
            )
        ))
        yaw_demo_deg = waypoint.get('yaw_deg')
        yaw_world_deg = (demo_yaw_to_world_deg((self.fx, self.fy), yaw_demo_deg)
                         if yaw_demo_deg is not None else None)
        rospy.logwarn(
            "[DEMO8] EGO P%d/%d: F%+.2f L%+.2f yaw_demo=%s yaw_world=%s",
            index, total, f, l, str(yaw_demo_deg), str(yaw_world_deg))

        p = self.odom.pose.pose.position

        goal = self.session["goal_world"]

        gx = float(goal["x"])
        gy = float(goal["y"])

        # EGO waypointCallback intentionally uses
        # current odom Z; use same altitude in message.
        gz = absolute_z

        xd, yd, hd = self.to_demo(
            (
                gx,
                gy,
                gz
            )
        )

        print()
        print("========================================")
        print(" AIRBORNE 5M EGO PREVIEW")
        print("========================================")
        print(
            "Current world = "
            "(%.3f, %.3f, %.3f)"
            %
            (
                p.x,
                p.y,
                p.z
            )
        )
        print(
            "Goal world    = "
            "(%.3f, %.3f, %.3f)"
            %
            (
                gx,
                gy,
                gz
            )
        )
        print(
            "Goal DEMO     = "
            "forward=%+.3f left=%+.3f "
            "height=%+.3f"
            %
            (
                xd,
                yd,
                hd
            )
        )
        print()
        print("PLAN -> VALIDATE -> DIRECT EXECUTE")
        print("NO pre-ALIGN")
        print("translation and tangent-yaw start together")
        print("NO software ARM")
        print("========================================")
        print()

        # TRAJ_SERVER_ACK_V1
        #
        # A stale ALIGN target must never survive into a new
        # planning attempt.
        stale_align = (
            self.root
            / "alignment_target.yaml"
        )

        if stale_align.exists():
            stale_align.unlink()

        baseline = self.bspline_count

        self.bspline_event.clear()

        msg = PoseStamped()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"

        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.position.z = gz

        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)

        deadline = time.time() + 8.0

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
        ):

            if (
                self.bspline_count > baseline
                and
                self.bspline is not None
            ):
                break

            time.sleep(0.05)

        if self.bspline_count <= baseline:
            raise RuntimeError(
                "No new EGO B-spline received"
            )

        # The next EGO curve exists. Stop QR BEFORE trajectory handoff/execution.
        self.stop_qr()

        # TRAJ_SERVER_ACK_V1
        #
        # Receiving the B-spline in THIS preview node is not
        # sufficient.  traj_server must also have loaded the
        # SAME traj_id and be publishing its frozen t=0 command.
        self.wait_traj_server_ack(
            int(self.bspline.traj_id)
        )

        # Safety assertion: preview must still be POINT.
        if self.mux_status != "POINT_ACTIVE":
            raise RuntimeError(
                "MUX changed during preview: %s"
                % self.mux_status
            )

        pts, u0, u1 = self.sample_bspline(
            self.bspline
        )

        # ------------------------------------------------------
        # HARD ALTITUDE VALIDATION
        #
        # The trajectory must actually live near the intended
        # airborne Demo height. A ground-level B-spline must
        # never proceed to ALIGN / EXECUTE.
        # ------------------------------------------------------

        target_h = float(
            self.session["goal_demo"]["height"]
        )

        demo_pts = [
            self.to_demo(p)
            for p in pts
        ]

        plan_h_min = min(
            q[2]
            for q in demo_pts
        )

        plan_h_max = max(
            q[2]
            for q in demo_pts
        )

        allowed_h_min = (
            target_h - 0.40
        )

        allowed_h_max = (
            target_h + 0.40
        )

        rospy.logwarn(
            "B-spline Demo-height range: "
            "[%.3f, %.3f] m; "
            "required inside [%.3f, %.3f] m",
            plan_h_min,
            plan_h_max,
            allowed_h_min,
            allowed_h_max
        )

        if (
            plan_h_min < allowed_h_min
            or
            plan_h_max > allowed_h_max
        ):

            raise RuntimeError(
                "B-spline altitude invalid: "
                "[%.3f, %.3f] m"
                %
                (
                    plan_h_min,
                    plan_h_max
                )
            )

        rospy.logwarn(
            "[ OK ] B-spline altitude accepted"
        )

        self.publish_path(
            pts
        )

        self.publish_tangents(
            pts
        )

        self.analyze(
            self.bspline,
            pts,
            self.bspline.traj_id,
            u0,
            u1
        )

        # RVIZ_REPLAN_TRAJ_V1
        self.publish_traj_marker(
            pts,
            int(self.bspline.traj_id)
        )

        rospy.logwarn(
            "PREVIEW VALIDATED. "
            "Starting DIRECT TANGENT execution."
        )

        try:

            self.execute_direct(
                int(self.bspline.traj_id),
                gx,
                gy,
                gz,
                land_after=(index == total),
                terminal_yaw_world_deg=yaw_world_deg
            )

        except Exception as e:
            rospy.logerr(
                "DIRECT TANGENT EXECUTION FAILED: %s",
                str(e)
            )

            try:
                self.restore_point_hold()
            except Exception as recovery_error:
                rospy.logerr(
                    "POINT HOLD recovery also failed: %s",
                    str(recovery_error)
                )

            self.yaw_mode_pub.publish(String(data="HOLD"))
            raise

        return

    def run(self):
        self.load_session()
        plan_path = self.root / "demo5_waypoints.yaml"
        self.demo5_cfg = load_config(plan_path)
        if self.demo5_cfg != self.session.get("demo5_plan_snapshot"):
            raise RuntimeError(
                "Demo5 YAML changed since DISARMED frame capture; ABORT"
            )
        self.segment_timeout_s = self.demo5_cfg["segment_timeout_s"]
        fence = self.demo5_cfg["fence"]
        for key, actual in (
            ("forward_min",self.x_min),("forward_max",self.x_max),
            ("left_min",self.y_min),("left_max",self.y_max),
            ("height_min",self.h_min),("height_max",self.h_max)
        ):
            if abs(fence[key] - actual) > 1e-7:
                raise RuntimeError(
                    "Demo5 session fence differs from YAML: " + key
                )
        # Initial flight settled in Demo3 POINT mode. Between every two
        # EGO legs the proven POINT abort service captures physical HOLD.
        self.wait_airborne_hold()
        self.check_goal_subscribers()
        total = len(self.demo5_cfg["waypoints"])
        for i, wp in enumerate(self.demo5_cfg["waypoints"], 1):
            if rospy.is_shutdown():
                raise RuntimeError("ROS shutdown before final waypoint")
            if i > 1:
                self.wait_airborne_hold(check_height=False)
            mode = wp["mode"]
            rospy.logwarn("[DEMO5 V2] leg %d/%d mode=%s", i, total, mode)
            if mode == "POINT":
                try:
                    self.execute_point_waypoint(i, total, wp)
                except Exception:
                    if (self.state is not None and
                            self.state.armed and self.state.mode == "OFFBOARD"):
                        rospy.logerr("[DEMO5 V2] POINT leg failed; restoring HOLD")
                        try:
                            self.restore_point_hold()
                        except Exception as recovery_error:
                            rospy.logerr("[WAYPOINT] HOLD recovery failed after original error: %s",
                                         recovery_error)
                    else:
                        rospy.logwarn("[DEMO5 V2] Not ARMED/OFFBOARD; skip airborne HOLD recovery")
                    raise
            elif mode == "EGO":
                self.plan_execute_waypoint(i, total, wp)
            else:
                raise RuntimeError("Unexpected waypoint mode: %s" % mode)

            # Start QR only after the selected waypoint has finished POINT HOLD.
            # It runs while hovering until the NEXT EGO B-spline is received.
            if (os.environ.get('DEMO10_QR_AFTER_WAYPOINT') == str(i)
                    and i < total
                    and self.demo5_cfg['waypoints'][i]['mode'] == 'EGO'):
                try:
                    self.qr_proc = subprocess.Popen([
                        'python3', '-B', '-u', str(self.root / 'demo7_qr_monitor.py')
                    ])
                    rospy.logwarn('[QR] recognition ON at P%d HOLD; awaiting P%d EGO curve', i, i+1)
                except OSError as exc:
                    rospy.logerr('[QR] observer could not start (flight unchanged): %s', exc)
        rospy.logwarn("[DEMO5 V2] ALL %d MIXED WAYPOINTS + LANDING COMPLETED",total)
        return


if __name__ == "__main__":

    rospy.init_node(
        "uav2_demo5_waypoints"
    )

    runner = TangentPreview()
    try:
        runner.run()
    finally:
        runner.stop_qr()  # Also stop QR if next EGO planning fails or ROS exits.
