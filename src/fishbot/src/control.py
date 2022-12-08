#!/usr/bin/env python3

import os

import rospy
from fishbot.msg import FishError


def callback(msg: FishError) -> None:
    folder = os.path.dirname(__file__)
    with open(os.path.join(folder, "num"), 'w', encoding="utf-8") as f:
        f.write(f"{msg.distance_error}\n{msg.angular_error}")
    
    linear_error = msg.distance_error
    angular_error = msg.angular_error

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


def controller() -> None:
    """
    Motion planning node.
    """
    rospy.Subscriber("/motion_plan", FishError, callback)
    rospy.spin()


def main() -> None:
    """
    Driver for motion planning node.
    """
    rospy.init_node("controller", anonymous=True)

    controller()


if __name__ == "__main__":
    main()
