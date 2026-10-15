#!/usr/bin/env python3

import math
import time
from pathlib import Path

import rospy
import yaml

from geometry_msgs.msg import Point
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


class WaypointPoleScene:

    def __init__(self):

        self.root = Path(__file__).resolve().parent

        self.mission_path = self.root / "mission.yaml"
        self.frame_path = self.root / "demo_frame.yaml"
        self.session_path = self.root / "session_frame.yaml"

        self.odom_topic = (
            "/contest2/mavros/local_position/odom"
        )

        self.load_config()
        self.load_basis()
        self.check_ground_state()
        self.capture_origin()
        self.validate_geometry()
        self.write_session()

        self.marker_pub = rospy.Publisher(
            "/contest2/yaw_tangent_demo/demo_frame_markers",
            MarkerArray,
            queue_size=1,
            latch=True
        )

        self.publish_scene()

        self.timer = rospy.Timer(
            rospy.Duration(1.0),
            self.timer_cb
        )

        print()
        print("========================================")
        print(" WAYPOINT POLE SCENE READY")
        print("========================================")
        print()
        print("This is VISUALIZATION ONLY.")
        print("NO ARM")
        print("NO OFFBOARD")
        print("NO flight setpoints")
        print("NO EGO goal")
        print()
        print("Check RViz geometry:")
        print(" P0 -> P1 -> P2 -> P3")
        print(" pole at F=1.0 L=2.0")
        print("========================================")


    # ============================================================
    # Configuration
    # ============================================================

    def load_config(self):

        with open(self.mission_path, "r") as f:
            self.cfg = yaml.safe_load(f)

        self.h = float(
            self.cfg["flight_height"]
        )

        fence = self.cfg["fence"]

        self.f_min = float(fence["forward_min"])
        self.f_max = float(fence["forward_max"])

        self.l_min = float(fence["left_min"])
        self.l_max = float(fence["left_max"])

        self.h_min = float(fence["height_min"])
        self.h_max = float(fence["height_max"])

        self.waypoints = self.cfg["waypoints"]
        self.obstacle = self.cfg["obstacle"]


    def load_basis(self):

        with open(self.frame_path, "r") as f:
            frame = yaml.safe_load(f)

        fw = frame["forward_world"]
        lw = frame["left_world"]

        self.fx = float(fw["x"])
        self.fy = float(fw["y"])

        self.lx = float(lw["x"])
        self.ly = float(lw["y"])

        nf = math.hypot(self.fx, self.fy)
        nl = math.hypot(self.lx, self.ly)

        dot = (
            self.fx * self.lx
            +
            self.fy * self.ly
        )

        if abs(nf - 1.0) > 0.03:
            raise RuntimeError(
                "FORWARD basis not unit length"
            )

        if abs(nl - 1.0) > 0.03:
            raise RuntimeError(
                "LEFT basis not unit length"
            )

        if abs(dot) > 0.03:
            raise RuntimeError(
                "F/L basis not orthogonal"
            )


    # ============================================================
    # Ground capture
    # ============================================================

    def check_ground_state(self):

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
                "Scene capture requires DISARMED"
            )

        if mission.data:
            raise RuntimeError(
                "Scene capture requires CH7 LOW"
            )


    def capture_origin(self):

        samples = []

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

            samples.append(
                (p.x, p.y, p.z)
            )

            if len(samples) >= 40:
                break

        if len(samples) < 20:
            raise RuntimeError(
                "Not enough odometry samples"
            )

        self.ox = sum(
            p[0] for p in samples
        ) / len(samples)

        self.oy = sum(
            p[1] for p in samples
        ) / len(samples)

        self.oz = sum(
            p[2] for p in samples
        ) / len(samples)

        print()
        print("========== NEW DEMO ORIGIN ==========")
        print(
            "WORLD = (%.3f, %.3f, %.3f)"
            % (
                self.ox,
                self.oy,
                self.oz
            )
        )
        print("DEMO  = F0.000 L0.000 H0.000")
        print("=====================================")


    # ============================================================
    # Coordinate conversion
    # ============================================================

    def demo_to_world(self, f, l, h):

        x = (
            self.ox
            +
            self.fx * f
            +
            self.lx * l
        )

        y = (
            self.oy
            +
            self.fy * f
            +
            self.ly * l
        )

        z = self.oz + h

        return x, y, z


    # ============================================================
    # Validation
    # ============================================================

    def inside(self, f, l, h):

        return (
            self.f_min <= f <= self.f_max
            and
            self.l_min <= l <= self.l_max
            and
            self.h_min <= h <= self.h_max
        )


    def margin(self, f, l, h):

        return min(
            f - self.f_min,
            self.f_max - f,
            l - self.l_min,
            self.l_max - l,
            h - self.h_min,
            self.h_max - h
        )


    @staticmethod
    def segment_distance(a, b, p):

        ax, ay = a
        bx, by = b
        px, py = p

        vx = bx - ax
        vy = by - ay

        wx = px - ax
        wy = py - ay

        vv = vx * vx + vy * vy

        if vv < 1e-9:
            return math.hypot(
                px - ax,
                py - ay
            )

        t = (
            wx * vx
            +
            wy * vy
        ) / vv

        t = max(0.0, min(1.0, t))

        qx = ax + t * vx
        qy = ay + t * vy

        return math.hypot(
            px - qx,
            py - qy
        )


    def validate_geometry(self):

        print()
        print("========== GEOMETRY CHECK ==========")

        points = [
            ("P0", 0.0, 0.0),
        ]

        for wp in self.waypoints:
            points.append(
                (
                    wp["name"],
                    float(wp["forward"]),
                    float(wp["left"])
                )
            )

        for name, f, l in points:

            ok = self.inside(
                f,
                l,
                self.h
            )

            m = self.margin(
                f,
                l,
                self.h
            )

            print(
                "%s : F=%+.2f L=%+.2f "
                "inside=%s margin=%.2f m"
                %
                (
                    name,
                    f,
                    l,
                    str(ok),
                    m
                )
            )

            if not ok:
                raise RuntimeError(
                    "%s outside Demo fence"
                    % name
                )

        of = float(
            self.obstacle["forward"]
        )

        ol = float(
            self.obstacle["left"]
        )

        if not self.inside(
            of,
            ol,
            self.h
        ):
            raise RuntimeError(
                "Pole outside Demo fence"
            )

        p1 = (
            float(self.waypoints[0]["forward"]),
            float(self.waypoints[0]["left"])
        )

        p2 = (
            float(self.waypoints[1]["forward"]),
            float(self.waypoints[1]["left"])
        )

        d = self.segment_distance(
            p1,
            p2,
            (of, ol)
        )

        print()
        print(
            "POLE : F=%+.2f L=%+.2f"
            % (of, ol)
        )

        print(
            "distance(POLE, nominal P1-P2)"
            " = %.3f m"
            % d
        )

        print("====================================")


    # ============================================================
    # Session file
    # ============================================================

    def write_session(self):

        data = {
            "origin_world": {
                "x": self.ox,
                "y": self.oy,
                "z": self.oz
            },

            "forward_world": {
                "x": self.fx,
                "y": self.fy
            },

            "left_world": {
                "x": self.lx,
                "y": self.ly
            },

            "flight_height": self.h,

            "demo_fence": self.cfg["fence"],

            "waypoints": self.waypoints,

            "obstacle": self.obstacle
        }

        with open(
            self.session_path,
            "w"
        ) as f:

            yaml.safe_dump(
                data,
                f,
                sort_keys=False
            )

        print()
        print(
            "[ OK ] session saved:"
        )
        print(self.session_path)


    # ============================================================
    # Markers
    # ============================================================

    @staticmethod
    def P(x, y, z):

        p = Point()
        p.x = x
        p.y = y
        p.z = z
        return p


    def add_text(
        self,
        arr,
        mid,
        text,
        x,
        y,
        z
    ):

        m = Marker()

        m.header.frame_id = "world"
        m.header.stamp = rospy.Time.now()

        m.ns = "waypoint_labels"
        m.id = mid

        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z

        m.pose.orientation.w = 1.0

        m.scale.z = 0.18

        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0

        m.text = text

        arr.markers.append(m)


    def publish_scene(self):

        arr = MarkerArray()

        now = rospy.Time.now()

        # --------------------------------------------------------
        # Delete old markers
        # --------------------------------------------------------

        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        # --------------------------------------------------------
        # Demo F/L axes
        # --------------------------------------------------------

        for idx, (
            name,
            df,
            dl,
            r,
            g,
            b
        ) in enumerate([
            ("F", 0.75, 0.0, 1.0, 0.2, 0.2),
            ("L", 0.0, 0.75, 0.2, 1.0, 0.2),
        ]):

            x0, y0, z0 = self.demo_to_world(
                0.0,
                0.0,
                0.05
            )

            x1, y1, z1 = self.demo_to_world(
                df,
                dl,
                0.05
            )

            m = Marker()

            m.header.frame_id = "world"
            m.header.stamp = now
            m.ns = "demo_axes"
            m.id = idx

            m.type = Marker.ARROW
            m.action = Marker.ADD

            m.points = [
                self.P(x0, y0, z0),
                self.P(x1, y1, z1)
            ]

            m.scale.x = 0.025
            m.scale.y = 0.07
            m.scale.z = 0.07

            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 1.0

            arr.markers.append(m)

            self.add_text(
                arr,
                100 + idx,
                name,
                x1,
                y1,
                z1 + 0.10
            )

        # --------------------------------------------------------
        # Horizontal Demo fence at flight height
        # --------------------------------------------------------

        corners_demo = [
            (self.f_min, self.l_min),
            (self.f_max, self.l_min),
            (self.f_max, self.l_max),
            (self.f_min, self.l_max),
            (self.f_min, self.l_min),
        ]

        fence = Marker()

        fence.header.frame_id = "world"
        fence.header.stamp = now
        fence.ns = "waypoint_demo_fence"
        fence.id = 0

        fence.type = Marker.LINE_STRIP
        fence.action = Marker.ADD

        fence.scale.x = 0.035

        fence.color.r = 1.0
        fence.color.g = 1.0
        fence.color.b = 0.0
        fence.color.a = 1.0

        for f, l in corners_demo:

            x, y, z = self.demo_to_world(
                f,
                l,
                self.h
            )

            fence.points.append(
                self.P(x, y, z)
            )

        arr.markers.append(fence)

        # --------------------------------------------------------
        # Nominal route P0-P1-P2-P3
        # --------------------------------------------------------

        route_demo = [
            ("P0", 0.0, 0.0)
        ]

        for wp in self.waypoints:

            route_demo.append(
                (
                    wp["name"],
                    float(wp["forward"]),
                    float(wp["left"])
                )
            )

        route = Marker()

        route.header.frame_id = "world"
        route.header.stamp = now
        route.ns = "nominal_route"
        route.id = 0

        route.type = Marker.LINE_STRIP
        route.action = Marker.ADD

        route.scale.x = 0.045

        route.color.r = 0.1
        route.color.g = 0.8
        route.color.b = 1.0
        route.color.a = 1.0

        for _, f, l in route_demo:

            x, y, z = self.demo_to_world(
                f,
                l,
                self.h
            )

            route.points.append(
                self.P(x, y, z)
            )

        arr.markers.append(route)

        # --------------------------------------------------------
        # Waypoints
        # --------------------------------------------------------

        for i, (name, f, l) in enumerate(
            route_demo
        ):

            x, y, z = self.demo_to_world(
                f,
                l,
                self.h
            )

            m = Marker()

            m.header.frame_id = "world"
            m.header.stamp = now
            m.ns = "waypoints"
            m.id = i

            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.pose.orientation.w = 1.0

            m.scale.x = 0.16
            m.scale.y = 0.16
            m.scale.z = 0.16

            if name == "P0":
                m.color.r = 0.3
                m.color.g = 1.0
                m.color.b = 0.3
            else:
                m.color.r = 0.2
                m.color.g = 0.5
                m.color.b = 1.0

            m.color.a = 1.0

            arr.markers.append(m)

            self.add_text(
                arr,
                200 + i,
                "%s  F%.1f L%.1f"
                % (name, f, l),
                x,
                y,
                z + 0.22
            )

        # --------------------------------------------------------
        # Pole
        # --------------------------------------------------------

        of = float(
            self.obstacle["forward"]
        )

        ol = float(
            self.obstacle["left"]
        )

        radius = float(
            self.obstacle["radius"]
        )

        height = float(
            self.obstacle["height"]
        )

        x, y, ground_z = self.demo_to_world(
            of,
            ol,
            0.0
        )

        pole = Marker()

        pole.header.frame_id = "world"
        pole.header.stamp = now

        pole.ns = "pole"
        pole.id = 0

        pole.type = Marker.CYLINDER
        pole.action = Marker.ADD

        pole.pose.position.x = x
        pole.pose.position.y = y
        pole.pose.position.z = (
            ground_z + height / 2.0
        )

        pole.pose.orientation.w = 1.0

        pole.scale.x = radius * 2.0
        pole.scale.y = radius * 2.0
        pole.scale.z = height

        pole.color.r = 1.0
        pole.color.g = 0.2
        pole.color.b = 0.1
        pole.color.a = 0.9

        arr.markers.append(pole)

        self.add_text(
            arr,
            300,
            "POLE  F1.0 L2.0",
            x,
            y,
            self.oz + self.h + 0.25
        )

        self.marker_pub.publish(arr)


    def timer_cb(self, event):
        self.publish_scene()


if __name__ == "__main__":

    rospy.init_node(
        "uav2_waypoint_scene_preview"
    )

    WaypointPoleScene()

    rospy.spin()
