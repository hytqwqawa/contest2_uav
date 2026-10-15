#!/usr/bin/env python3

from pathlib import Path

import rospy
import yaml

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


ROOT = Path(__file__).resolve().parent

MISSION = ROOT / "mission.yaml"
SESSION = ROOT / "session_frame.yaml"


def P(x, y, z):

    p = Point()
    p.x = x
    p.y = y
    p.z = z
    return p


with open(MISSION, "r") as f:
    mission = yaml.safe_load(f)

with open(SESSION, "r") as f:
    session = yaml.safe_load(f)

o = session["origin_world"]
fw = session["forward_world"]
lw = session["left_world"]

ox = float(o["x"])
oy = float(o["y"])
oz = float(o["z"])

fx = float(fw["x"])
fy = float(fw["y"])

lx = float(lw["x"])
ly = float(lw["y"])

h = float(mission["flight_height"])


def world(f, l, hh):

    return (
        ox + fx * f + lx * l,
        oy + fy * f + ly * l,
        oz + hh
    )


rospy.init_node(
    "uav2_waypoint_runtime_markers"
)

pub = rospy.Publisher(
    "/contest2/yaw_tangent_demo/demo_frame_markers",
    MarkerArray,
    queue_size=1,
    latch=True
)


# Clear all stale markers left by previous RViz/demo runs.
clear_array = MarkerArray()
clear_marker = Marker()
clear_marker.action = Marker.DELETEALL
clear_array.markers.append(clear_marker)

# Give the latched publisher a moment to connect.
rospy.sleep(0.3)
pub.publish(clear_array)
rospy.sleep(0.2)



def publish():

    arr = MarkerArray()

    now = rospy.Time.now()

    route = [
        ("P0", 0.0, 0.0)
    ]

    for wp in mission["waypoints"]:
        route.append(
            (
                wp["name"],
                float(wp["forward"]),
                float(wp["left"])
            )
        )


    for i, (name, f, l) in enumerate(route):

        x, y, z = world(f, l, h)

        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = now
        m.ns = "waypoint_pole_points"
        m.id = 1100 + i
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0

        m.scale.x = 0.16
        m.scale.y = 0.16
        m.scale.z = 0.16

        m.color.r = 0.2
        m.color.g = 0.55
        m.color.b = 1.0
        m.color.a = 1.0

        arr.markers.append(m)

        t = Marker()
        t.header.frame_id = "world"
        t.header.stamp = now
        t.ns = "waypoint_pole_labels"
        t.id = 1200 + i
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD

        t.pose.position.x = x
        t.pose.position.y = y
        t.pose.position.z = z + 0.22
        t.pose.orientation.w = 1.0

        t.scale.z = 0.16

        t.color.r = 1.0
        t.color.g = 1.0
        t.color.b = 1.0
        t.color.a = 1.0

        t.text = (
            "%s  F%.1f L%.1f"
            % (name, f, l)
        )

        arr.markers.append(t)

    obs = mission["obstacle"]

    of = float(obs["forward"])
    ol = float(obs["left"])

    r = float(obs["radius"])
    height = float(obs["height"])

    x, y, _ = world(of, ol, 0.0)

    pole = Marker()
    pole.header.frame_id = "world"
    pole.header.stamp = now
    pole.ns = "waypoint_pole_obstacle"
    pole.id = 1300
    pole.type = Marker.CYLINDER
    pole.action = Marker.ADD

    pole.pose.position.x = x
    pole.pose.position.y = y
    pole.pose.position.z = oz + height / 2.0
    pole.pose.orientation.w = 1.0

    pole.scale.x = 2.0 * r
    pole.scale.y = 2.0 * r
    pole.scale.z = height

    pole.color.r = 1.0
    pole.color.g = 0.15
    pole.color.b = 0.10
    pole.color.a = 1.0

    arr.markers.append(pole)

    pub.publish(arr)


publish()

timer = rospy.Timer(
    rospy.Duration(1.0),
    lambda event: publish()
)

rospy.spin()
