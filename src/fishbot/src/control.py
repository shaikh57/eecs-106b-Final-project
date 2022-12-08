#!/usr/bin/env python3

import os
from dataclasses import dataclass
import math

import rospy
from fishbot.msg import FishError


DISTANCE_THRESHOLD = 0.1
ANGULAR_THRESHOLD  = 0.15
MAX_SATURATION     = 50
MIN_SATURATION     = 20

class ESP32Command:
    pass

@dataclass
class DoneCommand(ESP32Command):

    def __str__(self) -> str:
        return "DONE\n0"

@dataclass
class ForwardCommand(ESP32Command):

    def __str__(self) -> str:
        return "FRWD\n0"

@dataclass
class TurnCommand(ESP32Command):
    strength: float

    def __str__(self) -> str:
        return f"TURN\n{self.strength}"

def callback(msg: FishError) -> None:
    folder = os.path.dirname(__file__)
  
    distance_err = msg.distance_error
    angular_err = msg.angular_error

    if distance_err <= DISTANCE_THRESHOLD:
        cmd = DoneCommand()
    elif distance_err > DISTANCE_THRESHOLD and abs(angular_err) <= ANGULAR_THRESHOLD:
        cmd = ForwardCommand()
    elif distance_err > DISTANCE_THRESHOLD and abs(angular_err) > ANGULAR_THRESHOLD:
        # Turning left is a negative angle
        # Turning right is a positive angle
        # facing 180 degrees away is 3 radians
        strength = -angular_err * 180 / math.pi
        sign = math.copysign(1, strength)
        if abs(strength) <= MIN_SATURATION:
            strength = MIN_SATURATION
        if abs(strength) >= MAX_SATURATION:
            strength = MAX_SATURATION
        cmd = TurnCommand(int(sign * strength))

    with open(os.path.join(folder, "num"), 'w', encoding="utf-8") as f:
        # when publishing to http sever, change "the f strings"
        f.write(str(cmd))


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
