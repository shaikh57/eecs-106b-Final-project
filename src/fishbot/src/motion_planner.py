#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def motion_planner() -> None:
    """
    Motion planning node.
    """
    pub = rospy.Publisher("motion_plan", String, queue_size=10)
    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        s = rospy.get_time()
        pub.publish(f"{s}")
        print(f"{rospy.get_name()} sent {s}")

        r.sleep()



def main() -> None:
    """
    Driver for motion planning node.
    """
    rospy.init_node("motion_planner", anonymous=True)

    motion_planner()


if __name__ == "__main__":
    main()