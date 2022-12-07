#!/usr/bin/env python3

import curses

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf2_ros
from geometry_msgs.msg import Twist

def motion_planner(scr: curses.window,) -> None:
    """
    Motion planning node.
    """
    #Create a publisher and a tf buffer, which is primed with a tf listener
    pub = rospy.Publisher('/motion_plan', Twist, queue_size=10)
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    #define fish frame and goal frame
    fish_frame = "frame1"
    goal_frame = "frame2"

    curses.noecho()
    scr.nodelay(True)
    pub = rospy.Publisher("motion_plan", String, queue_size=10)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        # s = rospy.get_time()
        try:
            # find the transform between the fish_frame and the goal_frame
            trans = tfBuffer.lookup_transform(fish_frame, goal_frame, rospy.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            #Gains
            K1 = 1
            K2 = 1

            control_command = Twist()
            control_command.linear.x = x*K1
            control_command.angular.z = y*K2

            s = scr.getkey()
        except curses.error:
            s = "s"
        pub.publish(control_command)
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