import os
from dataclasses import dataclass
import math
import numpy as np

import rospy
from fishbot.msg import FishError

def __init__(self, input_low_lims, input_high_lims):
    self.input_low_lims = input_low_lims
    self.input_high_lims = input_high_lims
    self.obstacles = []

def distance(self, c1, c2):
        """
        c1 and c2 should be numpy.ndarrays of size (4,)
        """
        c1_x = c1[0]
        c1_y = c1[1]
        c2_x = c2[0]
        c2_y = c2[1]
        theta1 = c1[2]
        theta2 = c2[2]
        dist = np.sqrt((c1_x-c2_x)**2 + (c1_y - c2_y)**2 + (min(abs(theta1 - theta2), 2*np.pi - abs(theta1-theta2)) % 2*np.pi)**2)
        return dist

def sample_config(self, *args):
        random = np.random.rand()
        if random > 0.7:
            return np.array(args[0])

        # xRand = np.random.uniform(self.low_lims[0], self.high_lims[0])
        # yRand = np.random.uniform(self.low_lims[1], self.high_lims[1])
        # thetaRand = np.random.uniform(self.low_lims[2], self.high_lims[2])
        # phiRand = np.random.uniform(-0.6, 0.6)
        # return np.array([xRand, yRand, thetaRand, phiRand])
        return np.random.uniform(self.low_lims, self.high_lims).reshape((self.dim,))

def check_collision(self, c):
        """
        Returns true if a configuration c is in collision
        c should be a numpy.ndarray of size (4,)
        """
        x = c[0]
        y = c[1]
        for obst in self.obstacles:
            if (x - obst[0])**2 + (y - obst[1])**2 <= (obst[2])**2:
                return True
        return False

def check_path_collision(self, path):
        """
        Returns true if the input path is in collision. The path
        is given as a Plan object. See configuration_space.py
        for details on the Plan interface.
        You should also ensure that the path does not exceed any state bounds,
        and the open loop inputs don't exceed input bounds.
        """
        for time in path.times:
            c, input = path.get(time)

            if self.check_collision(c):
                return True
            if input[0] < self.input_low_lims[0] or input[0] > self.input_high_lims[0] or input[1] < self.input_low_lims[1] or input[1] > self.input_high_lims[1]:
                return True
            if c[0] < self.low_lims[0] or c[0] > self.high_lims[0] or c[1] < self.low_lims[1] or c[1] > self.high_lims[1]:
                return True
        return False

def local_plan(self, c1, c2, dt=0.01):
    best_dist = np.inf
    best_plan = None

    for u1 in np.linspace(self.input_low_lims[0], self.input_high_lims[0], 20):
        for u2 in np.linspace(self.input_low_lims[1], self.input_high_lims[1], 10):
            total_time = 0.5
            # print(distance / self.input_high_lims[0])
            c1_upd = c1
            positions = [np.array(c1)]
            velocities = len(np.arange(0, total_time, dt))*[[u1, u2]]
            times = np.arange(0, total_time, dt)
            for t in np.arange(dt, total_time, dt):
                q_dot = np.array([np.cos(c1_upd[2]) * u1, np.sin(c1_upd[2]) * u1, (1/self.robot_length) * np.tan(c1_upd[3]) * u1, 0]) + np.array([0,0,0,u2])
                c1_upd = c1_upd + q_dot*dt
                positions.append(c1_upd)
            plan = Plan(times, positions, velocities, dt=dt)
            if self.distance(plan.end_position(), c2) < best_dist:
                best_dist = self.distance(plan.end_position(), c2)
                best_plan = plan
        # print("best_plan:, ", best_plan.positions)
        return best_plan