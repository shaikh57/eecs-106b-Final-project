#!/usr/bin/env python

"""
Starter code for EECS C106B Spring 2020 Project 2.
Author: Amay Saxena
"""
import sys
import time
# import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from collections import defaultdict
from configuration_space import FishConfigurationSpace, Plan
sys.setrecursionlimit(30000)
import cv2


class RRTGraph(object):

    def __init__(self, *nodes):
        self.nodes = [n for n in nodes]
        self.parent = defaultdict(lambda: None)
        self.path = defaultdict(lambda: None)
        self.rec = False
        
    def add_node(self, new_config, parent, path):
        new_config = tuple(new_config)
        parent = tuple(parent)
        # Stops a tree error where self.parent will have an almost infinte number of
        # key, value pairs that equal each other.
        if parent == new_config:
            return
        self.nodes.append(new_config)
        self.parent[new_config] = parent
        self.path[(parent, new_config)] = path

    def get_edge_paths(self):
        for pair in self.path:
            yield self.path[pair]

    def construct_path_to(self, c):
        path = []     
        while len(c) > 0:
            c = tuple(c)
            parent_c = self.parent[c]
            if not parent_c:
                break
            path.append(self.path[(parent_c, c)])
            c = parent_c
        if path:
            return Plan.chain_paths(*reversed(path))
        else:
            return None

class RRTPlanner(object):

    def __init__(self, config_space, max_iter=10000, expand_dist=0.3):
        # config_space should be an object of type ConfigurationSpace
        # (or a subclass of ConfigurationSpace).
        self.config_space = config_space
        # Maximum number of iterations to run RRT for:
        self.max_iter = max_iter
        # Exit the algorithm once a node is sampled within this 
        # distance of the goal:
        self.expand_dist = expand_dist

    def plan_to_pose(self, start, goal, dt=0.01, prefix_time_length=1):
        """
            Uses the RRT algorithm to plan from the start configuration
            to the goal configuration.
        """
        print("======= Planning with RRT =======")
        self.graph = RRTGraph(start)
        self.plan = None
        print("Iteration:", 0)
        # range(self.max_iter)
        for it in range(self.max_iter):
            sys.stdout.write("\033[F")
            print("Iteration:", it + 1)
            # if rospy.is_shutdown():
            #     print("Stopping path planner.")
            #     break
            rand_config = self.config_space.sample_config(goal)
            if self.config_space.check_collision(rand_config):
                continue
            closest_config = self.config_space.nearest_config_to(self.graph.nodes, rand_config)
            path = self.config_space.local_plan(closest_config, rand_config)
            if self.config_space.check_path_collision(path):
                continue
            delta_path = path.get_prefix(prefix_time_length)
            new_config = delta_path.end_position()
            self.graph.add_node(new_config, closest_config, delta_path)
            if self.config_space.distance(new_config, goal) <= self.expand_dist:
                path_to_goal = self.config_space.local_plan(new_config, goal)
                if self.config_space.check_path_collision(path_to_goal):
                    continue
                self.graph.add_node(goal, new_config, path_to_goal)
                self.plan = self.graph.construct_path_to(goal)
                return self.plan
        print("Failed to find plan in allotted number of iterations.")
        return None

    def plot_execution(self):
        """
        Creates a plot of the RRT graph on the environment. Assumes that the 
        environment of the robot is in the x-y plane, and that the first two
        components in the state space are x and y position. Also assumes 
        plan_to_pose has been called on this instance already, so that self.graph
        is populated. If planning was successful, then self.plan will be populated 
        and it will be plotted as well.
        """
        ax = plt.subplot(1, 1, 1)
        ax.set_aspect(1)
        ax.set_xlim(self.config_space.low_lims[0], self.config_space.high_lims[0])
        ax.set_ylim(self.config_space.low_lims[1], self.config_space.high_lims[1])

        for obs in self.config_space.obstacles:
            x, y, x_w, y_h = obs
            ax.add_patch(Rectangle((x, y), x_w - x, y_h - y))

        for path in self.graph.get_edge_paths():
            xs = []
            ys = []
            for pos in path.positions:
                xs.append(pos[0])
                ys.append(pos[1])
            ax.plot(xs, ys, color='orange')

        if self.plan:
            plan_x = []
            plan_y = []
            for pos in self.plan.positions:
                plan_x.append(pos[0])
                plan_y.append(pos[1])
            ax.plot(plan_x, plan_y, color='green')

        plt.show()

def main():
    """Use this function if you'd like to test without ROS.

    If you're testing at home without ROS, you might want
    to get rid of the rospy.is_shutdown check in the main 
    planner loop (and the corresponding rospy import).
    """
    # state = [x, y, phi]
    # phi is fishbot heading angle

    # input = [Turning_angle (theta), frequency (tail turn rate)]
    # theta limited to +/- 0.785 rad (45 degrees)

    start = np.array([1, 1, 0]) 
    goal = np.array([9, 9, 0.785])
    xy_low = [0, 0]
    xy_high = [10, 10]
    phi_max = 0.785 # 360 degrees

    # u1 = Turning_angle
    u1_max = 0.8 # 45 degrees
    # u2 = frequency
    u2_max = 1 # [rad/s]
    
    # obstacles = [[6, 3.5, 0.5], [3.5, 6.5, 0.5]]
    # obstacles = []
    # obstacles = [[4,4,1.5]]
    cap = cv2.VideoCapture(1)
    count = 0
    obstacles = []
    while count < 1:
        # Read a frame from the webcam
        count += 1
        ret, frame = cap.read()
        resize_img = cv2.resize(frame  , (640 , 480))
        # Convert the frame to grayscale
        gray = cv2.cvtColor(resize_img, cv2.COLOR_BGR2GRAY)

        # Detect edges in the grayscale frame using the Canny algorithm
        edges = cv2.Canny(gray, 100, 200)

        # Find contours in the edges image
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Loop over all contours and filter for rectangular shapes
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w)/h
                if aspect_ratio >= 0.8 and aspect_ratio <= 1.2 and (w) > 5 and (h) > 5:
                    cv2.rectangle(resize_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    obstacles.append([x*10/640,y*10/480,(x+w)*10/640,(y+h)*10/480])   
    cv2.imshow('frame', resize_img)           
    cap.release()
    print(obstacles)
    config = FishConfigurationSpace( xy_low + [0],
                                        xy_high + [phi_max],
                                        [-u1_max, 0],
                                        [u1_max, u2_max],
                                        obstacles,
                                        0.15)
    # expand_dist = 1.5 works for no obstacles
    planner = RRTPlanner(config, max_iter=10000, expand_dist=2)
    plan = planner.plan_to_pose(start, goal)
    planner.plot_execution()

if __name__ == '__main__':
    main()