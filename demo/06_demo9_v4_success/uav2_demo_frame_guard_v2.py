#!/usr/bin/env python3

import math
import time
from pathlib import Path

import rospy
import yaml

from geometry_msgs.msg import Point
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from demo5_config import load_config, demo_to_world


class DemoFrameGuardV2:

    def __init__(self):

        # ============================================================
        # ROS / configuration
        # ============================================================

        self.odom_topic = rospy.get_param(
            "~odom_topic",
            "/contest2/mavros/local_position/odom"
        )

        self.frame_yaml = Path(
            rospy.get_param(
                "~demo_frame_yaml",
                str(
                    Path(__file__).resolve().parent
                    / "demo_frame.yaml"
                )
            )
        )

        # Authoritative DEMO-FRAME fence
        # OBSTACLE_DEMO_FORWARD_EXPAND_V1
        #
        # Allow EGO obstacle avoidance to use lateral bypass
        # motion while keeping Demo-frame safety supervision.
        self.x_min = rospy.get_param("~x_min", -1.50)
        self.x_max = rospy.get_param("~x_max", +1.50)

        self.y_min = rospy.get_param("~y_min", -0.50)
        self.y_max = rospy.get_param("~y_max", +5.50)

        # Ground monitoring allowance
        self.h_min = rospy.get_param("~h_min", -0.25)
        self.h_max = rospy.get_param("~h_max", +1.30)

        # EGO should only receive airborne goals.
        self.ego_h_min = rospy.get_param(
            "~ego_h_min",
            +0.10
        )

        self.ego_h_max = rospy.get_param(
            "~ego_h_max",
            +1.50
        )

        # Small numerical allowance for legacy AABB.
        self.aabb_padding = rospy.get_param(
            "~aabb_padding",
            0.05
        )

        self.goal_left = rospy.get_param(
            "~goal_left",
            5.00
        )

        self.nominal_flight_height = rospy.get_param(
            "~flight_height",
            0.80
        )

        # DEMO5: authoritative bounds + ordered waypoints share one YAML.
        # A private ROS param cannot silently override this fixed fence.
        plan_path = self.frame_yaml.parent / "demo5_waypoints.yaml"
        self.demo5_cfg = load_config(plan_path)
        fence = self.demo5_cfg["fence"]
        self.x_min = fence["forward_min"]
        self.x_max = fence["forward_max"]
        self.y_min = fence["left_min"]
        self.y_max = fence["left_max"]
        self.h_min = fence["height_min"]
        self.h_max = fence["height_max"]
        self.nominal_flight_height = self.demo5_cfg["flight_height"]
        final = self.demo5_cfg["waypoints"][-1]
        self.goal_forward = final["forward"]
        self.goal_left = final["left"]

        self.odom = None
        self.odom_time = None
        self.origin = None

        self.load_basis()

        # ============================================================
        # Safety: this session frame MUST be captured on the ground
        # ============================================================

        print()
        print("========================================")
        print(" UAV2 DEMO FRAME GUARD V2")
        print("========================================")
        print()
        print("This node:")
        print("  - captures current takeoff origin")
        print("  - defines relative rotated fence")
        print("  - generates EGO-compatible WORLD AABB")
        print("  - draws fence in RViz")
        print()
        print("It does NOT:")
        print("  - ARM")
        print("  - change flight mode")
        print("  - publish flight setpoints")
        print()

        state = rospy.wait_for_message(
            "/contest2/mavros/state",
            State,
            timeout=5.0
        )

        mission = rospy.wait_for_message(
            "/contest2/ego_uav_interface/rc_mission_allowed",
            Bool,
            timeout=5.0
        )

        if state.armed:
            raise RuntimeError(
                "Guard V2 must be started while DISARMED"
            )

        if mission.data:
            raise RuntimeError(
                "Guard V2 requires CH7 LOW "
                "when capturing the Demo origin"
            )

        # ============================================================
        # Publishers
        # ============================================================

        self.safe_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/demo_frame_safe",
            Bool,
            queue_size=1,
            latch=True
        )

        self.status_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/demo_frame_guard_status",
            String,
            queue_size=1,
            latch=True
        )

        self.ready_pub = rospy.Publisher(
            "/contest2/yaw_tangent_demo/session_ready",
            Bool,
            queue_size=1,
            latch=True
        )

        self.marker_pub = rospy.Publisher(
            "/contest2/yaw_tangent_demo/demo_frame_markers",
            MarkerArray,
            queue_size=1,
            latch=True
        )

        # Reuse the topics already present in the proven RViz config.
        self.heading_pub = rospy.Publisher(
            "/contest2/uav2_static/body_heading",
            Marker,
            queue_size=1
        )

        self.goal_pub = rospy.Publisher(
            "/contest2/uav2_static/goal_marker",
            Marker,
            queue_size=1,
            latch=True
        )

        rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_cb,
            queue_size=50
        )

        self.capture_origin()

        # Convert the rotated Demo-frame box to a conservative
        # axis-aligned WORLD box for legacy EGO compatibility.
        self.configure_legacy_arena_aabb()

        self.write_session_yaml()

        self.publish_static_markers()

        self.ready_pub.publish(
            Bool(data=True)
        )

        self.timer = rospy.Timer(
            rospy.Duration(0.10),
            self.timer_cb
        )

        self.marker_timer = rospy.Timer(
            rospy.Duration(1.0),
            self.marker_timer_cb
        )


    # ================================================================
    # Demo basis
    # ================================================================

    def load_basis(self):

        if not self.frame_yaml.exists():
            raise RuntimeError(
                "Missing demo_frame.yaml: %s"
                % self.frame_yaml
            )

        with open(self.frame_yaml, "r") as f:
            cfg = yaml.safe_load(f)

        self.fx = float(
            cfg["forward_world"]["x"]
        )
        self.fy = float(
            cfg["forward_world"]["y"]
        )

        self.lx = float(
            cfg["left_world"]["x"]
        )
        self.ly = float(
            cfg["left_world"]["y"]
        )

        nf = math.hypot(
            self.fx,
            self.fy
        )

        nl = math.hypot(
            self.lx,
            self.ly
        )

        dot = (
            self.fx * self.lx
            +
            self.fy * self.ly
        )

        if abs(nf - 1.0) > 0.03:
            raise RuntimeError(
                "FORWARD basis is not unit length"
            )

        if abs(nl - 1.0) > 0.03:
            raise RuntimeError(
                "LEFT basis is not unit length"
            )

        if abs(dot) > 0.03:
            raise RuntimeError(
                "Demo basis is not orthogonal"
            )


    # ================================================================
    # Odom / origin
    # ================================================================

    def odom_cb(self, msg):

        self.odom = msg
        self.odom_time = rospy.Time.now()


    def capture_origin(self):

        samples = []
        yaw_samples = []

        deadline = time.time() + 8.0

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
        ):

            try:
                msg = rospy.wait_for_message(
                    self.odom_topic,
                    Odometry,
                    timeout=1.0
                )
            except rospy.ROSException:
                continue

            p = msg.pose.pose.position
            q = msg.pose.pose.orientation

            # Yaw of current aircraft body +X in WORLD.
            siny_cosp = 2.0 * (
                q.w * q.z
                +
                q.x * q.y
            )

            cosy_cosp = 1.0 - 2.0 * (
                q.y * q.y
                +
                q.z * q.z
            )

            yaw = math.atan2(
                siny_cosp,
                cosy_cosp
            )

            samples.append(
                (
                    p.x,
                    p.y,
                    p.z
                )
            )

            yaw_samples.append(yaw)

            if len(samples) >= 40:
                break

        if len(samples) < 20:
            raise RuntimeError(
                "Not enough odometry samples "
                "to capture Demo session frame"
            )

        ox = sum(
            v[0] for v in samples
        ) / len(samples)

        oy = sum(
            v[1] for v in samples
        ) / len(samples)

        oz = sum(
            v[2] for v in samples
        ) / len(samples)

        # Circular mean -- do not average angles directly.
        sy = sum(
            math.sin(y)
            for y in yaw_samples
        )

        cy = sum(
            math.cos(y)
            for y in yaw_samples
        )

        yaw = math.atan2(
            sy,
            cy
        )

        max_yaw_dev = max(
            abs(
                math.atan2(
                    math.sin(y - yaw),
                    math.cos(y - yaw)
                )
            )
            for y in yaw_samples
        )

        max_yaw_dev_deg = math.degrees(
            max_yaw_dev
        )

        if max_yaw_dev_deg > 5.0:
            raise RuntimeError(
                "Heading unstable while capturing "
                "Demo frame: max deviation %.2f deg"
                % max_yaw_dev_deg
            )

        self.origin = (
            ox,
            oy,
            oz
        )

        # IMPORTANT:
        # At Demo-frame startup the DISARMED aircraft nose must
        # physically point toward NEW FIELD FORWARD (= old LEFT).
        #
        # Therefore current body +X defines +X_D for THIS
        # T265 session.  +Y_D is its left-hand in-plane axis.
        self.fx = math.cos(yaw)
        self.fy = math.sin(yaw)

        self.lx = -self.fy
        self.ly = +self.fx

        print("========== DEMO SESSION FRAME ==========")

        print(
            "WORLD origin = "
            "(%.3f, %.3f, %.3f)"
            %
            (
                ox,
                oy,
                oz
            )
        )

        print(
            "captured NEW FORWARD (nose) yaw = "
            "%+.3f deg"
            %
            math.degrees(yaw)
        )

        print(
            "FORWARD world = "
            "(%+.6f, %+.6f)"
            %
            (
                self.fx,
                self.fy
            )
        )

        print(
            "LEFT world    = "
            "(%+.6f, %+.6f)"
            %
            (
                self.lx,
                self.ly
            )
        )

        print(
            "heading sample max deviation = "
            "%.3f deg"
            %
            max_yaw_dev_deg
        )

        print(
            "DEMO origin = "
            "(0.000, 0.000, 0.000)"
        )

        print("========================================")


    # ================================================================
    # Coordinate transform
    # ================================================================

    def world_to_demo(
        self,
        wx,
        wy,
        wz
    ):

        ox, oy, oz = self.origin

        dx = wx - ox
        dy = wy - oy

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

        hd = wz - oz

        return (
            xd,
            yd,
            hd
        )


    def demo_to_world(
        self,
        xd,
        yd,
        hd
    ):

        ox, oy, oz = self.origin

        wx = (
            ox
            +
            self.fx * xd
            +
            self.lx * yd
        )

        wy = (
            oy
            +
            self.fy * xd
            +
            self.ly * yd
        )

        wz = oz + hd

        return (
            wx,
            wy,
            wz
        )


    # ================================================================
    # Compatibility AABB for existing EGO source
    # ================================================================

    def configure_legacy_arena_aabb(self):

        corners = []

        for xd in [
            self.x_min,
            self.x_max
        ]:

            for yd in [
                self.y_min,
                self.y_max
            ]:

                wx, wy, _ = self.demo_to_world(
                    xd,
                    yd,
                    0.0
                )

                corners.append(
                    (
                        wx,
                        wy
                    )
                )

        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]

        ax0 = (
            min(xs)
            -
            self.aabb_padding
        )

        ax1 = (
            max(xs)
            +
            self.aabb_padding
        )

        ay0 = (
            min(ys)
            -
            self.aabb_padding
        )

        ay1 = (
            max(ys)
            +
            self.aabb_padding
        )

        az0 = (
            self.origin[2]
            +
            self.ego_h_min
        )

        az1 = (
            self.origin[2]
            +
            self.ego_h_max
        )

        rospy.set_param(
            "/contest2/arena/safe/enabled",
            True
        )

        rospy.set_param(
            "/contest2/arena/safe/x_min",
            ax0
        )

        rospy.set_param(
            "/contest2/arena/safe/x_max",
            ax1
        )

        rospy.set_param(
            "/contest2/arena/safe/y_min",
            ay0
        )

        rospy.set_param(
            "/contest2/arena/safe/y_max",
            ay1
        )

        rospy.set_param(
            "/contest2/arena/safe/z_min",
            az0
        )

        rospy.set_param(
            "/contest2/arena/safe/z_max",
            az1
        )

        self.aabb = (
            ax0,
            ax1,
            ay0,
            ay1,
            az0,
            az1
        )

        print()
        print("========== EGO COMPATIBILITY AABB ==========")
        print(
            "x = [%.3f, %.3f]"
            %
            (
                ax0,
                ax1
            )
        )

        print(
            "y = [%.3f, %.3f]"
            %
            (
                ay0,
                ay1
            )
        )

        print(
            "z = [%.3f, %.3f]"
            %
            (
                az0,
                az1
            )
        )

        print(
            "NOTE: this AABB is NOT the authoritative fence."
        )

        print(
            "The rotated Demo-frame fence remains authoritative."
        )

        print("=============================================")


    # ================================================================
    # Session file
    # ================================================================

    def write_session_yaml(self):

        gx, gy, gz = self.demo_to_world(
            self.goal_forward,
            self.goal_left,
            self.nominal_flight_height
        )

        data = {

            # This binds RViz preview, guard and executor to ONE YAML.
            "demo5_plan_snapshot": self.demo5_cfg,
            "demo5_waypoints": self.demo5_cfg["waypoints"],
            "origin_world": {
                "x": float(
                    self.origin[0]
                ),
                "y": float(
                    self.origin[1]
                ),
                "z": float(
                    self.origin[2]
                ),
            },

            "forward_world": {
                "x": float(self.fx),
                "y": float(self.fy),
            },

            "left_world": {
                "x": float(self.lx),
                "y": float(self.ly),
            },

            "demo_fence": {
                "forward_min": float(
                    self.x_min
                ),
                "forward_max": float(
                    self.x_max
                ),
                "left_min": float(
                    self.y_min
                ),
                "left_max": float(
                    self.y_max
                ),
                "height_min": float(
                    self.h_min
                ),
                "height_max": float(
                    self.h_max
                ),
            },

            "goal_demo": {
                "forward": float(self.goal_forward),
                "left": float(
                    self.goal_left
                ),
                "height": float(
                    self.nominal_flight_height
                ),
            },

            "goal_world": {
                "x": float(gx),
                "y": float(gy),
                "z": float(gz),
            },

            "ego_compatibility_aabb": {
                "x_min": float(
                    self.aabb[0]
                ),
                "x_max": float(
                    self.aabb[1]
                ),
                "y_min": float(
                    self.aabb[2]
                ),
                "y_max": float(
                    self.aabb[3]
                ),
                "z_min": float(
                    self.aabb[4]
                ),
                "z_max": float(
                    self.aabb[5]
                ),
            },
        }

        path = (
            Path(__file__).resolve().parent
            /
            "session_frame.yaml"
        )

        with open(path, "w") as f:
            yaml.safe_dump(
                data,
                f,
                sort_keys=False
            )

        print()
        print("[ OK ] session frame saved:")
        print(path)

        print()
        print(
            "FINAL WAYPOINT world = "
            "(%.3f, %.3f, %.3f)"
            %
            (
                gx,
                gy,
                gz
            )
        )


    # ================================================================
    # Safety check
    # ================================================================

    def check_safe(
        self,
        xd,
        yd,
        hd
    ):

        return (
            self.x_min
            <= xd
            <= self.x_max
            and
            self.y_min
            <= yd
            <= self.y_max
            and
            self.h_min
            <= hd
            <= self.h_max
        )


    def min_margin(
        self,
        xd,
        yd,
        hd
    ):

        return min(
            xd - self.x_min,
            self.x_max - xd,
            yd - self.y_min,
            self.y_max - yd,
            hd - self.h_min,
            self.h_max - hd,
        )


    # ================================================================
    # RViz markers
    # ================================================================

    @staticmethod
    def point(
        x,
        y,
        z
    ):

        p = Point()

        p.x = x
        p.y = y
        p.z = z

        return p


    def publish_static_markers(self):

        now = rospy.Time.now()

        arr = MarkerArray()

        # ------------------------------------------------------------
        # Actual rotated Demo fence -- 3D wireframe
        # ------------------------------------------------------------

        z0 = self.origin[2]
        z1 = (
            self.origin[2]
            +
            self.h_max
        )

        lower = []
        upper = []

        corners_demo = [
            (
                self.x_min,
                self.y_min
            ),
            (
                self.x_max,
                self.y_min
            ),
            (
                self.x_max,
                self.y_max
            ),
            (
                self.x_min,
                self.y_max
            ),
        ]

        for xd, yd in corners_demo:

            wx, wy, _ = self.demo_to_world(
                xd,
                yd,
                0.0
            )

            lower.append(
                self.point(
                    wx,
                    wy,
                    z0
                )
            )

            upper.append(
                self.point(
                    wx,
                    wy,
                    z1
                )
            )

        fence = Marker()

        fence.header.frame_id = "world"
        fence.header.stamp = now

        fence.ns = "demo_fence"
        fence.id = 0

        fence.type = Marker.LINE_LIST
        fence.action = Marker.ADD

        fence.scale.x = 0.035

        fence.color.r = 1.0
        fence.color.g = 0.85
        fence.color.b = 0.0
        fence.color.a = 1.0

        # lower + upper rectangles
        for ring in [
            lower,
            upper
        ]:

            for i in range(4):

                fence.points.append(
                    ring[i]
                )

                fence.points.append(
                    ring[
                        (i + 1) % 4
                    ]
                )

        # vertical edges
        for i in range(4):

            fence.points.append(
                lower[i]
            )

            fence.points.append(
                upper[i]
            )

        arr.markers.append(fence)

        # ------------------------------------------------------------
        # FORWARD axis
        # ------------------------------------------------------------

        forward = Marker()

        forward.header.frame_id = "world"
        forward.header.stamp = now

        forward.ns = "demo_axes"
        forward.id = 1

        forward.type = Marker.ARROW
        forward.action = Marker.ADD

        forward.points = [
            self.point(
                self.origin[0],
                self.origin[1],
                self.origin[2] + 0.05
            ),
            self.point(
                self.origin[0] + self.fx,
                self.origin[1] + self.fy,
                self.origin[2] + 0.05
            ),
        ]

        forward.scale.x = 0.04
        forward.scale.y = 0.10
        forward.scale.z = 0.10

        forward.color.r = 1.0
        forward.color.g = 0.25
        forward.color.b = 0.15
        forward.color.a = 1.0

        arr.markers.append(forward)

        # ------------------------------------------------------------
        # LEFT axis
        # ------------------------------------------------------------

        left = Marker()

        left.header.frame_id = "world"
        left.header.stamp = now

        left.ns = "demo_axes"
        left.id = 2

        left.type = Marker.ARROW
        left.action = Marker.ADD

        left.points = [
            self.point(
                self.origin[0],
                self.origin[1],
                self.origin[2] + 0.05
            ),
            self.point(
                self.origin[0] + self.lx,
                self.origin[1] + self.ly,
                self.origin[2] + 0.05
            ),
        ]

        left.scale.x = 0.04
        left.scale.y = 0.10
        left.scale.z = 0.10

        left.color.r = 0.15
        left.color.g = 1.0
        left.color.b = 0.30
        left.color.a = 1.0

        arr.markers.append(left)

        # ------------------------------------------------------------
        # Labels
        # ------------------------------------------------------------

        labels = [
            (
                3,
                "FORWARD +X_D",
                self.origin[0] + 1.10 * self.fx,
                self.origin[1] + 1.10 * self.fy,
            ),
            (
                4,
                "LEFT +Y_D",
                self.origin[0] + 1.10 * self.lx,
                self.origin[1] + 1.10 * self.ly,
            ),
        ]

        for mid, text, x, y in labels:

            m = Marker()

            m.header.frame_id = "world"
            m.header.stamp = now

            m.ns = "demo_labels"
            m.id = mid

            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD

            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = (
                self.origin[2]
                +
                0.12
            )

            m.pose.orientation.w = 1.0

            m.scale.z = 0.18

            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0

            m.text = text

            arr.markers.append(m)

        # Demo5 mixed modes: label EACH waypoint with POINT or EGO.
        # POINT segments are straight and do NOT provide obstacle avoidance.
        # EGO segments use the existing avoidance/replanning pipeline.
        for i, wp in enumerate(self.demo5_cfg["waypoints"], 1):
            px, py, pz = self.demo_to_world(
                wp["forward"], wp["left"], self.nominal_flight_height
            )
            sphere = Marker()
            sphere.header.frame_id = "world"
            sphere.header.stamp = now
            sphere.ns = "demo5_waypoints"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.orientation.w = 1.0
            sphere.pose.position.x = px
            sphere.pose.position.y = py
            sphere.pose.position.z = pz
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.18
            sphere.color.a = 1.0
            if i == len(self.demo5_cfg["waypoints"]):
                sphere.color.r = 0.2
                sphere.color.g = 0.45
                sphere.color.b = 1.0
            else:
                sphere.color.r = 1.0
                sphere.color.g = 0.35
                sphere.color.b = 0.15
            arr.markers.append(sphere)
            label = Marker()
            label.header = sphere.header
            label.ns = "demo5_waypoint_labels"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.orientation.w = 1.0
            label.pose.position.x = px
            label.pose.position.y = py
            label.pose.position.z = pz + 0.28
            label.scale.z = 0.20
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = "P%d %s F%+.1f L%+.1f%s" % (
                i, wp["mode"], wp["forward"], wp["left"],
                " FINAL" if i == len(self.demo5_cfg["waypoints"]) else ""
            )
            arr.markers.append(label)

        self.marker_pub.publish(arr)

        # ------------------------------------------------------------
        # Goal marker -- reuse proven RViz topic
        # ------------------------------------------------------------

        gx, gy, gz = self.demo_to_world(
            self.goal_forward,
            self.goal_left,
            self.nominal_flight_height
        )

        goal = Marker()

        goal.header.frame_id = "world"
        goal.header.stamp = now

        goal.ns = "yaw_tangent_goal"
        goal.id = 0

        goal.type = Marker.SPHERE
        goal.action = Marker.ADD

        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = gz

        goal.pose.orientation.w = 1.0

        goal.scale.x = 0.28
        goal.scale.y = 0.28
        goal.scale.z = 0.28

        goal.color.r = 0.0
        goal.color.g = 0.3
        goal.color.b = 1.0
        goal.color.a = 1.0

        self.goal_pub.publish(goal)


    def marker_timer_cb(self, event):

        self.publish_static_markers()


    def publish_heading(self):

        if self.odom is None:
            return

        m = Marker()

        m.header.frame_id = "world"
        m.header.stamp = rospy.Time.now()

        m.ns = "uav_heading"
        m.id = 0

        m.type = Marker.ARROW
        m.action = Marker.ADD

        m.pose = self.odom.pose.pose

        m.scale.x = 0.55
        m.scale.y = 0.12
        m.scale.z = 0.12

        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0

        self.heading_pub.publish(m)


    # ================================================================
    # Runtime monitor
    # ================================================================

    def timer_cb(self, event):

        if self.odom is None:

            self.safe_pub.publish(
                Bool(data=False)
            )

            return

        age = (
            rospy.Time.now()
            -
            self.odom_time
        ).to_sec()

        if age > 0.30:

            self.safe_pub.publish(
                Bool(data=False)
            )

            self.status_pub.publish(
                String(
                    data=(
                        "STALE_ODOM age=%.3f"
                        % age
                    )
                )
            )

            return

        p = self.odom.pose.pose.position

        xd, yd, hd = self.world_to_demo(
            p.x,
            p.y,
            p.z
        )

        safe = self.check_safe(
            xd,
            yd,
            hd
        )

        margin = self.min_margin(
            xd,
            yd,
            hd
        )

        state = (
            "SAFE"
            if safe
            else "UNSAFE"
        )

        text = (
            "%s "
            "| DEMO forward=%+.3f "
            "left=%+.3f "
            "height=%+.3f "
            "| margin=%+.3f"
            %
            (
                state,
                xd,
                yd,
                hd,
                margin
            )
        )

        self.safe_pub.publish(
            Bool(data=safe)
        )

        self.status_pub.publish(
            String(data=text)
        )

        self.publish_heading()

        if safe:

            rospy.loginfo_throttle(
                0.50,
                text
            )

        else:

            rospy.logwarn_throttle(
                0.20,
                text
            )


if __name__ == "__main__":

    rospy.init_node(
        "contest2_demo_frame_guard_v2"
    )

    try:

        DemoFrameGuardV2()

        rospy.spin()

    except Exception as e:

        rospy.logfatal(
            "Demo Frame Guard V2 failed: %s",
            str(e)
        )

        raise
