#!/usr/bin/env python3

import curses
import itertools
import math
from dataclasses import dataclass, field
from typing import Iterable
import time

import rospy
from fishbot.msg import FishError
from geometry_msgs.msg import Pose, PoseStamped, Point
from std_msgs.msg import String
import tf.transformations as tft

# Calibrated Corners of the Tank
front_left = Pose(position=Point(x=0.31, y=-0.25))
front_right = Pose(position=Point(x=-0.23, y=-0.24))
back_left = Pose(position=Point(x=0.34, y=0.18))
back_right = Pose(position=Point(x=-0.23, y=0.18))

target1 = back_right
target2 = Pose(position=Point(x=0.05, y=0.05))
target3 = front_left
target4 = back_left

@dataclass
class TagTracker:
    fish_tag:       Pose     = field(default_factory=Pose)
    target_tag:     Pose     = field(default_factory=Pose)
    distance_error: float    = float("inf")
    angular_error:  float    = float("inf")
    control_pts:    Iterable = itertools.cycle([
        target1,
        target2,
        target3,
        target4
    ])

    def __post_init__(self):
        self.target_tag = next(self.control_pts)

    def fish_tag_callback(self, fish_tag: PoseStamped) -> None:
        self.fish_tag = fish_tag.pose
        self.joint_callback()

    def target_tag_callback(self) -> None:
        if abs(self.distance_error) <= 0.1:
            self.target_tag = next(self.control_pts)
        self.joint_callback()

    def joint_callback(self) -> None:
        delta_x = self.fish_tag.position.x - self.target_tag.position.x
        delta_y = self.fish_tag.position.y - self.target_tag.position.y
        fish_quaternion = [
            self.fish_tag.orientation.x,
            self.fish_tag.orientation.y,
            self.fish_tag.orientation.z,
            self.fish_tag.orientation.w
        ]
        _, _, yaw = tft.euler_from_quaternion(fish_quaternion)

        self.distance_error = math.sqrt(delta_x**2 + delta_y**2)
        self.angular_error = math.atan2(delta_y, delta_x) - yaw

        if self.angular_error < -math.pi:
            self.angular_error += 2 * math.pi
        elif self.angular_error > math.pi:
            # Want to bound the right turn angular error at pi
            # Want to bound the left turn angular at -pi
            self.angular_error -= 2 * math.pi


def motion_planner() -> None:
    """
    Motion planning node.
    """
    tag_tracker = TagTracker()
    # rospy.Subscriber("/aruco_target/pose", PoseStamped, tag_tracker.target_tag_callback)
    rospy.Subscriber("/aruco_fish/pose", PoseStamped, tag_tracker.fish_tag_callback)
    pub = rospy.Publisher("/motion_plan", FishError, queue_size=10)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        tag_tracker.target_tag_callback()
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
    time.sleep(5)
    rospy.init_node("motion_planner", anonymous=True)

    motion_planner()
    # curses.wrapper(teleop)


if __name__ == "__main__":
    main()