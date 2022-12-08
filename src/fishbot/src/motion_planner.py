#!/usr/bin/env python3

import curses
import math
from dataclasses import dataclass, field

import rospy
from fishbot.msg import FishError
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
import tf.transformations as tft


@dataclass
class TagTracker:
    fish_tag:       Pose  = field(default_factory=Pose)
    target_tag:     Pose  = field(default_factory=Pose)
    distance_error: float = 0
    angular_error:  float = 0

    def fish_tag_callback(self, fish_tag: PoseStamped) -> None:
        self.fish_tag = fish_tag.pose
        self.joint_callback()

    def target_tag_callback(self, target_tag: PoseStamped) -> None:
        self.target_tag = target_tag.pose
        self.joint_callback()

    def joint_callback(self) -> None:
        delta_x = self.fish_tag.position.x - self.target_tag.position.x
        delta_y = self.fish_tag.position.y - self.target_tag.position.y
        fish_quaternion = [self.fish_tag.orientation.x, self.fish_tag.orientation.y, self.fish_tag.orientation.z, self.fish_tag.orientation.w]
        _, _, yaw = tft.euler_from_quaternion(fish_quaternion)

        self.distance_error = math.sqrt(delta_x**2 + delta_y**2)
        self.angular_error = math.atan2(delta_y, delta_x) - yaw


def motion_planner() -> None:
    """
    Motion planning node.
    """
    tag_tracker = TagTracker()
    rospy.Subscriber("/aruco_target/pose", PoseStamped, tag_tracker.target_tag_callback)
    rospy.Subscriber("/aruco_fish/pose", PoseStamped, tag_tracker.fish_tag_callback)
    pub = rospy.Publisher('/motion_plan', FishError, queue_size=10)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        #Gains
        K1 = 1
        K2 = 1
        fish_error = FishError(tag_tracker.distance_error, tag_tracker.angular_error)
        pub.publish(fish_error)
        r.sleep()


def teleop(scr: curses.window) -> None:
    """
    DEBUGGING ONLY
    """
    curses.noecho()
    scr.nodelay(True)
    pub = rospy.Publisher("/motion_plan", String, queue_size=10)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            s = scr.getkey()
        except curses.error:
            s = "s"
        pub.publish(s)

        r.sleep()


def main() -> None:
    """
    Driver for motion planning node.
    """
    rospy.init_node("motion_planner", anonymous=True)

    motion_planner()
    # curses.wrapper(teleop)


if __name__ == "__main__":
    main()