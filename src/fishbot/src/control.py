#!/usr/bin/env python3

import os
import random

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def callback(msg: String) -> None:
    folder = os.path.dirname(__file__)
    with open(os.path.join(folder, "num"), 'w', encoding="utf-8") as f:
        # f.write(f"{rospy.get_name()} received {msg.data}")
        f.write(str(random.randint(0, 9)))
    print(f"{rospy.get_name()} received {msg.data}")


def controller() -> None:
    """
    Motion planning node.
    """
    rospy.Subscriber("motion_plan", String, callback)

    rospy.spin()


def main() -> None:
    """
    Driver for motion planning node.
    """
    rospy.init_node("controller", anonymous=True)

    controller()


if __name__ == "__main__":
    main()