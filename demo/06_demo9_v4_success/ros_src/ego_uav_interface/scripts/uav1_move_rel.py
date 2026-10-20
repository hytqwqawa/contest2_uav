#!/usr/bin/env python3

import sys
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


ODOM_TOPIC = "/uav1/mavros/local_position/odom"
GOAL_TOPIC = "/ego_uav_interface/goal"


def main():

    rospy.init_node(
        "uav1_move_rel_cli",
        anonymous=True
    )

    if len(sys.argv) != 3:
        print(
            "Usage: rosrun ego_uav_interface "
            "uav1_move_rel.py DX DY"
        )
        sys.exit(1)

    try:
        dx = float(sys.argv[1])
        dy = float(sys.argv[2])
    except ValueError:
        print("DX and DY must be numbers.")
        sys.exit(1)

    distance = (dx * dx + dy * dy) ** 0.5

    # Extra CLI-side safety limit.
    # point_traj_node itself still retains its independent 3 m limit.
    if distance > 1.0:
        print(
            "[ABORT] Relative command %.2f m exceeds "
            "1.0 m test-command limit." % distance
        )
        sys.exit(1)

    print("Waiting for current odometry...")

    try:
        odom = rospy.wait_for_message(
            ODOM_TOPIC,
            Odometry,
            timeout=2.0
        )
    except rospy.ROSException:
        print("[ABORT] No fresh odometry.")
        sys.exit(1)

    x0 = odom.pose.pose.position.x
    y0 = odom.pose.pose.position.y
    z0 = odom.pose.pose.position.z

    goal = PoseStamped()

    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = x0 + dx
    goal.pose.position.y = y0 + dy

    # point_traj_node currently ignores goal z and keeps
    # the current mission altitude, but populate it anyway.
    goal.pose.position.z = z0

    goal.pose.orientation.w = 1.0

    pub = rospy.Publisher(
        GOAL_TOPIC,
        PoseStamped,
        queue_size=1,
        latch=True
    )

    # Give ROS publisher time to establish connections.
    rospy.sleep(0.3)

    print(
        "Current : [%.3f, %.3f, %.3f]" %
        (x0, y0, z0)
    )

    print(
        "Goal    : [%.3f, %.3f, %.3f]" %
        (
            goal.pose.position.x,
            goal.pose.position.y,
            goal.pose.position.z
        )
    )

    print(
        "Relative: dx=%.3f, dy=%.3f, distance=%.3f m" %
        (dx, dy, distance)
    )

    pub.publish(goal)

    rospy.sleep(0.5)

    print("Goal published.")


if __name__ == "__main__":
    main()
