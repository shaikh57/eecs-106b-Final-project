#!/usr/bin/env python3

import os

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def callback(msg: String) -> None:
    folder = os.path.dirname(__file__)
    with open(os.path.join(folder, "num"), 'w', encoding="utf-8") as f:
        f.write(str(msg.data))
    print(f"{rospy.get_name()} received {msg.data}")


def controller() -> None:
    """
    Motion planning node.
    """
    rospy.Subscriber("motion_plan", String, callback)

    ### take in linear (in cm) and angular error (in degrees)
    linear_error = 0
    angular_error = 0

    command = ''

    ### first case: STOP
    if linear_error <= 2:     # tune
        # publish STOP command
        command = 'S'

    ### second case: go STRAIGHT
    elif angular_error <= 5:     # tune
        # publish GO_STRAIGHT command
        command = 'G'

    ### third case: TURN
    else:     # angular_error > 5
        Kp = 1
        # publish TURN command with gain proportional to angular error

    rospy.spin()


def main() -> None:
    """
    Driver for motion planning node.
    """
    rospy.init_node("controller", anonymous=True)

    controller()


if __name__ == "__main__":
    main()
