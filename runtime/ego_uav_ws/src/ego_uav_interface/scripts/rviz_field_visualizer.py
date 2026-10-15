#!/usr/bin/env python3

import math
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion


# ============================================================
# Flight boundaries
# ============================================================

HARD_X_MIN = -3.0
HARD_X_MAX =  3.5
HARD_Y_MIN = -1.7
HARD_Y_MAX =  1.7

PLAN_X_MIN = -2.8
PLAN_X_MAX =  3.3
PLAN_Y_MIN = -1.5
PLAN_Y_MAX =  1.5

FRAME_ID = "world"


class FieldVisualizer:

    def __init__(self):

        self.odom = None

        # Latest RViz-selected and accepted goal
        self.goal_valid = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_z = 0.0
        self.goal_yaw = 0.0

        self.pub = rospy.Publisher(
            "/ego_uav_interface/field_markers",
            MarkerArray,
            queue_size=1,
            latch=True
        )

        rospy.Subscriber(
            "/uav1/mavros/local_position/odom",
            Odometry,
            self.odom_cb,
            queue_size=20
        )

        # Use the goal accepted by our RViz safety executor.
        rospy.Subscriber(
            "/ego_uav_interface/active_rviz_goal",
            PoseStamped,
            self.goal_cb,
            queue_size=5
        )

        rospy.Timer(
            rospy.Duration(0.05),
            self.timer_cb
        )

        rospy.loginfo("======================================")
        rospy.loginfo(" RViz field visualizer READY")
        rospy.loginfo(" Hard fence: x=[%.1f %.1f], y=[%.1f %.1f]",
                      HARD_X_MIN, HARD_X_MAX,
                      HARD_Y_MIN, HARD_Y_MAX)
        rospy.loginfo(" Plan fence: x=[%.1f %.1f], y=[%.1f %.1f]",
                      PLAN_X_MIN, PLAN_X_MAX,
                      PLAN_Y_MIN, PLAN_Y_MAX)
        rospy.loginfo("======================================")


    # ========================================================
    # Callbacks
    # ========================================================

    def odom_cb(self, msg):

        self.odom = msg


    def goal_cb(self, msg):

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_z = msg.pose.position.z

        q = msg.pose.orientation

        _, _, self.goal_yaw = euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])

        self.goal_valid = True

        rospy.logwarn(
            "[RVIZ GOAL DISPLAY] x=%.3f y=%.3f yaw=%.1f deg",
            self.goal_x,
            self.goal_y,
            math.degrees(self.goal_yaw)
        )


    # ========================================================
    # Marker helpers
    # ========================================================

    def make_box(
        self,
        marker_id,
        xmin,
        xmax,
        ymin,
        ymax,
        z,
        width,
        r,
        g,
        b
    ):

        m = Marker()

        m.header.frame_id = FRAME_ID
        m.header.stamp = rospy.Time.now()

        m.ns = "flight_boundary"
        m.id = marker_id

        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD

        m.pose.orientation.w = 1.0

        m.scale.x = width

        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0

        for x, y in [
            (xmin, ymin),
            (xmax, ymin),
            (xmax, ymax),
            (xmin, ymax),
            (xmin, ymin)
        ]:

            p = Point()
            p.x = x
            p.y = y
            p.z = z

            m.points.append(p)

        return m


    def make_arrow(
        self,
        marker_id,
        namespace,
        x,
        y,
        z,
        yaw,
        length,
        shaft,
        r,
        g,
        b
    ):

        m = Marker()

        m.header.frame_id = FRAME_ID
        m.header.stamp = rospy.Time.now()

        m.ns = namespace
        m.id = marker_id

        m.type = Marker.ARROW
        m.action = Marker.ADD

        # For ARROW defined by two points:
        # scale.x = shaft diameter
        # scale.y = head diameter
        # scale.z = head length
        m.scale.x = shaft
        m.scale.y = shaft * 2.3
        m.scale.z = shaft * 3.0

        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0

        p0 = Point()
        p0.x = x
        p0.y = y
        p0.z = z

        p1 = Point()
        p1.x = x + length * math.cos(yaw)
        p1.y = y + length * math.sin(yaw)
        p1.z = z

        m.points = [p0, p1]

        return m


    def make_text(
        self,
        marker_id,
        namespace,
        text,
        x,
        y,
        z,
        size,
        r=1.0,
        g=1.0,
        b=1.0
    ):

        m = Marker()

        m.header.frame_id = FRAME_ID
        m.header.stamp = rospy.Time.now()

        m.ns = namespace
        m.id = marker_id

        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z

        m.pose.orientation.w = 1.0

        m.scale.z = size

        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0

        m.text = text

        return m


    def make_sphere(
        self,
        marker_id,
        namespace,
        x,
        y,
        z,
        diameter,
        r,
        g,
        b
    ):

        m = Marker()

        m.header.frame_id = FRAME_ID
        m.header.stamp = rospy.Time.now()

        m.ns = namespace
        m.id = marker_id

        m.type = Marker.SPHERE
        m.action = Marker.ADD

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0

        m.scale.x = diameter
        m.scale.y = diameter
        m.scale.z = diameter

        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0

        return m


    # ========================================================
    # Main visualization
    # ========================================================

    def timer_cb(self, event):

        arr = MarkerArray()

        # ----------------------------------------------------
        # Hard fence
        # ----------------------------------------------------

        arr.markers.append(
            self.make_box(
                0,
                HARD_X_MIN,
                HARD_X_MAX,
                HARD_Y_MIN,
                HARD_Y_MAX,
                0.03,
                0.045,
                1.0, 0.15, 0.15
            )
        )

        # ----------------------------------------------------
        # Planning-safe fence
        # ----------------------------------------------------

        arr.markers.append(
            self.make_box(
                1,
                PLAN_X_MIN,
                PLAN_X_MAX,
                PLAN_Y_MIN,
                PLAN_Y_MAX,
                0.05,
                0.025,
                1.0, 0.75, 0.0
            )
        )

        # ----------------------------------------------------
        # WORLD +X
        # ----------------------------------------------------

        arr.markers.append(
            self.make_arrow(
                10,
                "world_axes",
                0.0, 0.0, 0.08,
                0.0,
                0.80,
                0.035,
                1.0, 0.0, 0.0
            )
        )

        arr.markers.append(
            self.make_text(
                11,
                "world_axes_text",
                "+X",
                0.92, 0.0, 0.15,
                0.18,
                1.0, 0.2, 0.2
            )
        )

        # ----------------------------------------------------
        # WORLD +Y
        # ----------------------------------------------------

        arr.markers.append(
            self.make_arrow(
                12,
                "world_axes",
                0.0, 0.0, 0.08,
                math.pi / 2.0,
                0.80,
                0.035,
                0.0, 1.0, 0.0
            )
        )

        arr.markers.append(
            self.make_text(
                13,
                "world_axes_text",
                "+Y",
                0.0, 0.92, 0.15,
                0.18,
                0.2, 1.0, 0.2
            )
        )

        # ----------------------------------------------------
        # Boundary labels
        # ----------------------------------------------------

        arr.markers.append(
            self.make_text(
                20,
                "boundary_text",
                "HARD FENCE",
                HARD_X_MIN + 0.65,
                HARD_Y_MAX - 0.12,
                0.12,
                0.14
            )
        )

        arr.markers.append(
            self.make_text(
                21,
                "boundary_text",
                "EGO SAFE AREA",
                PLAN_X_MIN + 0.80,
                PLAN_Y_MAX - 0.15,
                0.13,
                0.14
            )
        )

        # ----------------------------------------------------
        # UAV position + nose direction
        # ----------------------------------------------------

        if self.odom is not None:

            p = self.odom.pose.pose.position
            q = self.odom.pose.pose.orientation

            _, _, yaw = euler_from_quaternion([
                q.x, q.y, q.z, q.w
            ])

            # UAV center
            arr.markers.append(
                self.make_sphere(
                    30,
                    "uav",
                    p.x,
                    p.y,
                    p.z,
                    0.13,
                    0.0, 0.65, 1.0
                )
            )

            # Nose/body +X direction
            arr.markers.append(
                self.make_arrow(
                    31,
                    "uav_heading",
                    p.x,
                    p.y,
                    p.z + 0.04,
                    yaw,
                    0.45,
                    0.045,
                    0.0, 0.70, 1.0
                )
            )

            arr.markers.append(
                self.make_text(
                    32,
                    "uav_heading_text",
                    "NOSE\nyaw=%.1f deg" %
                    math.degrees(yaw),
                    p.x + 0.55 * math.cos(yaw),
                    p.y + 0.55 * math.sin(yaw),
                    p.z + 0.16,
                    0.13,
                    0.0, 0.8, 1.0
                )
            )

        # ----------------------------------------------------
        # Latest accepted RViz goal
        # ----------------------------------------------------

        if self.goal_valid:

            # Our executor makes the target altitude equal to
            # current flight altitude.
            display_z = self.goal_z

            if display_z < 0.1 and self.odom is not None:
                display_z = self.odom.pose.pose.position.z

            # Goal center
            arr.markers.append(
                self.make_sphere(
                    40,
                    "rviz_goal",
                    self.goal_x,
                    self.goal_y,
                    display_z,
                    0.16,
                    1.0, 0.0, 1.0
                )
            )

            # Goal yaw arrow
            arr.markers.append(
                self.make_arrow(
                    41,
                    "rviz_goal",
                    self.goal_x,
                    self.goal_y,
                    display_z + 0.04,
                    self.goal_yaw,
                    0.55,
                    0.045,
                    1.0, 0.0, 1.0
                )
            )

            # Numeric goal information
            goal_text = (
                "GOAL\n"
                "x = %.2f m\n"
                "y = %.2f m\n"
                "yaw = %.1f deg"
                %
                (
                    self.goal_x,
                    self.goal_y,
                    math.degrees(self.goal_yaw)
                )
            )

            arr.markers.append(
                self.make_text(
                    42,
                    "rviz_goal_text",
                    goal_text,
                    self.goal_x,
                    self.goal_y,
                    display_z + 0.38,
                    0.14,
                    1.0, 0.3, 1.0
                )
            )

        self.pub.publish(arr)


if __name__ == "__main__":

    rospy.init_node(
        "rviz_field_visualizer"
    )

    FieldVisualizer()

    rospy.spin()
