#!/usr/bin/env python3

import math
import time
import threading
import subprocess
import yaml
from pathlib import Path

import rospy
from geometry_msgs.msg import Point
import rosgraph

from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from traj_utils.msg import Bspline
from quadrotor_msgs.msg import PositionCommand


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class TangentPreview:

    def __init__(self):

        self.root = Path(__file__).resolve().parent

        self.session_path = (
            self.root
            / "session_frame.yaml"
        )

        self.odom = None
        self.state = None

        self.point_status = ""
        self.mux_status = ""
        self.demo_safe = False

        # DIRECT_TANGENT_EXEC_V1
        self.flight_healthy = False
        self.mission_allowed = False

        self.cmd = None
        self.cmd_time = None

        # RVIZ_REPLAN_TRAJ_V1
        self.traj_marker_pub = rospy.Publisher(
            "/contest2/uav2/replan_traj",
            Marker,
            queue_size=5,
            latch=True
        )

        # REALTIME_TRAJ_RVIZ_V1
        # Continuous commanded trajectory. This naturally contains
        # the path actually followed across EGO replans.
        self.executed_traj_pub = rospy.Publisher(
            "/contest2/uav2/executed_traj",
            Marker,
            queue_size=1,
            latch=True
        )

        self.executed_traj_points = []

        # ACTUAL_TRAJ_RVIZ_V1
        # Real aircraft trajectory from PX4 odometry.
        self.actual_traj_pub = rospy.Publisher(
            "/contest2/uav2/actual_traj",
            Marker,
            queue_size=1,
            latch=True
        )

        self.actual_traj_points = []

        self.bspline = None
        self.bspline_count = 0
        self.bspline_event = threading.Event()

        self.goal_pub = rospy.Publisher(
            "/contest2/move_base_simple/goal",
            PoseStamped,
            queue_size=1
        )

        self.path_pub = rospy.Publisher(
            "/contest2/uav2_static/ego_path",
            RosPath,
            queue_size=1,
            latch=True
        )

        self.tangent_pub = rospy.Publisher(
            "/contest2/yaw_tangent_demo/tangent_markers",
            MarkerArray,
            queue_size=1,
            latch=True
        )

        # DIRECT_TANGENT_EXEC_V1
        self.yaw_mode_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/yaw_mode",
            String,
            queue_size=1,
            latch=True
        )

        self.trigger_pub = rospy.Publisher(
            "/contest2/traj_start_trigger",
            PoseStamped,
            queue_size=1
        )

        rospy.Subscriber(
            "/contest2/mavros/local_position/odom",
            Odometry,
            self.odom_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/mavros/state",
            State,
            self.state_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/point_traj_status",
            String,
            self.point_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/trajectory_mux_status",
            String,
            self.mux_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/demo_frame_safe",
            Bool,
            self.safe_cb,
            queue_size=10
        )

        # DIRECT_TANGENT_EXEC_V1
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
            "/contest2/position_cmd",
            PositionCommand,
            self.cmd_cb,
            queue_size=100
        )

        # ACTUAL_TRAJ_RVIZ_V1
        rospy.Subscriber(
            "/contest2/mavros/local_position/odom",
            Odometry,
            self.actual_odom_cb,
            queue_size=50
        )

        rospy.Subscriber(
            "/contest2/drone_0_planning/bspline",
            Bspline,
            self.bspline_cb,
            queue_size=10
        )

        rospy.loginfo(
            "Yaw-tangent direct demo ready. "
            "PLAN -> VALIDATE -> MOVE + TANGENT."
        )


    def odom_cb(self, msg):
        self.odom = msg


    def state_cb(self, msg):
        self.state = msg


    def point_cb(self, msg):
        self.point_status = msg.data


    def mux_cb(self, msg):
        self.mux_status = msg.data


    def safe_cb(self, msg):
        self.demo_safe = msg.data


    # DIRECT_TANGENT_EXEC_V1
    def health_cb(self, msg):
        self.flight_healthy = msg.data


    def mission_cb(self, msg):
        self.mission_allowed = msg.data



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
                    "/contest2/position_cmd",
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
                time.time() - first_time >= 0.30
            ):

                if max_move >= 0.03:
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
            "traj_id=%d on /contest2/position_cmd"
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
                "/contest2/drone_0_traj_server/"
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

            if topic == "/contest2/move_base_simple/goal":
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
                "/contest2/move_base_simple/goal -- unsafe"
            )

        if not any(
            "ego_planner" in n
            for n in goal_subs
        ):
            raise RuntimeError(
                "EGO goal subscriber missing"
            )


    def wait_airborne_hold(self):

        target_h = float(
            self.session["goal_demo"]["height"]
        )

        height_tol = rospy.get_param(
            "~preview_height_tolerance",
            0.08
        )

        max_vz = rospy.get_param(
            "~preview_max_vz",
            0.10
        )

        stable_time = rospy.get_param(
            "~preview_height_stable_time",
            0.80
        )

        stable_since = None

        rospy.loginfo(
            "Waiting for PHYSICAL airborne settle:"
        )

        rospy.loginfo(
            "  target Demo height = %.2f m",
            target_h
        )

        rospy.loginfo(
            "  height tolerance   = +/- %.2f m",
            height_tol
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

            height_ok = (
                abs(
                    hd - target_h
                )
                <= height_tol
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

        rospy.logwarn(
            "[DIRECT] Restoring POINT HOLD..."
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



    # AUTO_LAND_PROVEN_CHAIN_V2
    def start_land(self):

        rospy.logwarn(
            "[DIRECT] Goal reached. "
            "Starting proven automatic landing chain..."
        )

        script = (
            Path(__file__).resolve().parent
            / "uav2_flight_ops_dev.py"
        )

        if not script.exists():

            raise RuntimeError(
                "landing script not found: %s"
                % str(script)
            )

        rospy.logwarn(
            "[DIRECT] AUTO_LAND_PROVEN_CHAIN_V2: %s land",
            str(script)
        )

        # This process NEVER arms the vehicle.
        # The proven landing script performs:
        # POINT handoff
        # -> start_land
        # -> touchdown confirmation
        # -> freeze downward command
        # -> Demo3 final landing handoff
        # -> wait for PX4 auto-disarm.
        ret = subprocess.call(
            [
                "python3",
                str(script),
                "land"
            ]
        )

        if ret != 0:

            raise RuntimeError(
                "uav2_flight_ops_dev.py land failed "
                "with return code %d"
                % ret
            )

        rospy.logwarn(
            "[DIRECT] AUTOMATIC LANDING COMPLETE"
        )


    def execute_direct(
        self,
        traj_id,
        gx,
        gy,
        gz
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
            gap_xy >= 0.10
            or
            gap_z >= 0.10
        ):
            raise RuntimeError(
                "pre-release first gap >= 0.10 m"
            )

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
            "/contest2/ego_uav_interface/disable_trajectory"
        )

        # Select the already validated EGO command.
        self.call_service(
            "/contest2/ego_uav_interface/use_ego"
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
                "/contest2/traj_start_trigger does not have "
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
            post_gap_xy >= 0.10
            or
            post_gap_z >= 0.10
        ):
            raise RuntimeError(
                "post-trigger first gap >= 0.10 m"
            )

        # ----------------------------------------------------
        # RELEASE:
        #
        # position/velocity/acceleration and TANGENT yaw become
        # active at the SAME time.
        # ----------------------------------------------------

        self.call_service(
            "/contest2/ego_uav_interface/enable_trajectory"
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

        start_wall = time.time()
        deadline = start_wall + 60.0

        rate = rospy.Rate(20)

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
        ):

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

            if not self.flight_healthy:
                raise RuntimeError(
                    "flight_healthy=False during execution"
                )

            if not self.mission_allowed:
                raise RuntimeError(
                    "mission permission lost"
                )

            if not self.demo_safe:
                raise RuntimeError(
                    "Demo-frame guard UNSAFE"
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

            arrived = (
                dxy < 0.15
                and
                dz < 0.10
                and
                speed < 0.20
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
                "stable=%.2f/0.80"
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

            if stable >= 0.80:

                print()

                rospy.logwarn(
                    "[DIRECT] 5M GOAL REACHED"
                )

                # Capture final position as normal POINT hold.
                self.yaw_mode_pub.publish(
                    String(data="HOLD")
                )

                self.restore_point_hold()

                rospy.sleep(1.0)

                # AUTO_LAND_AFTER_GOAL_V1
                self.start_land()

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
            "direct 5-m execution timeout"
        )


    def run(self):

        self.load_session()

        self.wait_airborne_hold()

        self.check_goal_subscribers()

        p = self.odom.pose.pose.position

        goal = self.session["goal_world"]

        gx = float(goal["x"])
        gy = float(goal["y"])

        # EGO waypointCallback intentionally uses
        # current odom Z; use same altitude in message.
        gz = p.z

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
            target_h - 0.25
        )

        allowed_h_max = (
            target_h + 0.25
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
                gz
            )

        except Exception as e:

            rospy.logerr(
                "DIRECT TANGENT EXECUTION FAILED: %s",
                str(e)
            )

            self.restore_point_hold()

            raise

        rospy.spin()


if __name__ == "__main__":

    rospy.init_node(
        "uav2_tangent_preview"
    )

    TangentPreview().run()
