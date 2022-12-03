#!/usr/bin/env python3

import curses

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def motion_planner(scr: curses.window) -> None:
    """
    Motion planning node.
    """
    curses.noecho()
    scr.nodelay(True)
    pub = rospy.Publisher("motion_plan", String, queue_size=10)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        # s = rospy.get_time()
        try:
            s = scr.getkey()
        except curses.error:
            s = "s"
        pub.publish(s)
        print(f"{rospy.get_name()} sent {s}")

        r.sleep()



def main() -> None:
    """
    Driver for motion planning node.
    """
    rospy.init_node("motion_planner", anonymous=True)

    curses.wrapper(motion_planner)


if __name__ == "__main__":
    main()