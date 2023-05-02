#!/usr/bin/env python

"""
Starter code for EECS C106B Spring 2020 Project 2.
Author: Amay Saxena
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from contextlib import contextmanager
import sys
sys.setrecursionlimit(5000)

class Plan(object):
    """Data structure to represent a motion plan. Stores plans in the form of
    three arrays of the same length: times, positions, and open_loop_inputs.

    The following invariants are assumed:
        - at time times[i] the plan prescribes that we be in position
          positions[i] and perform input open_loop_inputs[i].
        - times starts at zero. Each plan is meant to represent the motion
          from one point to another over a time interval starting at 
          time zero. If you wish to append together multiple paths
          c1 -> c2 -> c3 -> ... -> cn, you should use the chain_paths
          method.
    """

    def __init__(self, times, target_positions, open_loop_inputs, dt=0.01):
        self.dt = dt
        self.times = times
        self.positions = target_positions
        self.open_loop_inputs = open_loop_inputs

    def __iter__(self):
        # I have to do this in an ugly way because python2 sucks and
        # I hate it.
        for t, p, c in zip(self.times, self.positions, self.open_loop_inputs):
            yield t, p, c

    def __len__(self):
        return len(self.times)

    def get(self, t):
        """Returns the desired position and open loop input at time t.
        """
        index = int(np.sum(self.times <= t))
        index = index - 1 if index else 0
        return self.positions[index], self.open_loop_inputs[index]

    def end_position(self):
        return self.positions[-1]

    def start_position(self):
        return self.positions[0]

    def get_prefix(self, until_time):
        """Returns a new plan that is a prefix of this plan up until the
        time until_time.
        """
        maxIndex = None
        for i in range(len(self.times)):
            maxIndex = i
            if self.times[i] > until_time:
                break
        times = self.times[0:maxIndex]
        positions = self.positions[0:maxIndex]
        open_loop_inputs = self.open_loop_inputs[0:maxIndex]
        return Plan(times, positions, open_loop_inputs)

    @classmethod
    def chain_paths(self, *paths):
        """Chain together any number of plans into a single plan.
        """
        def chain_two_paths(path1, path2):
            """Chains together two plans to create a single plan. Requires
            that path1 ends at the same configuration that path2 begins at.
            Also requires that both paths have the same discretization time
            step dt.
            """
            if not path1 and not path2:
                return None
            elif not path1:
                return path2
            elif not path2:
                return path1
            assert path1.dt == path2.dt, "Cannot append paths with different time deltas."
            assert np.allclose(path1.end_position(), path2.start_position()), "Cannot append paths with inconsistent start and end positions."
            times = np.concatenate((path1.times, path1.times[-1] + path2.times[1:]), axis=0)
            positions = np.concatenate((path1.positions, path2.positions[1:]), axis=0)
            open_loop_inputs = np.concatenate((path1.open_loop_inputs, path2.open_loop_inputs[1:]), axis=0)
            dt = path1.dt
            return Plan(times, positions, open_loop_inputs, dt=dt)
        chained_path = None
        for path in paths:
            chained_path = chain_two_paths(chained_path, path)
        return chained_path

@contextmanager
def expanded_obstacles(obstacle_list, delta):
    """Context manager that edits obstacle list to increase the radius of
    all obstacles by delta.
    
    Assumes obstacles are circles in the x-y plane and are given as lists
    of [x, y, r] specifying the center and radius of the obstacle. So
    obstacle_list is a list of [x, y, r] lists.

    Note we want the obstacles to be lists instead of tuples since tuples
    are immutable and we would be unable to change the radii.

    Usage:
        with expanded_obstacles(obstacle_list, 0.1):
            # do things with expanded obstacle_list. While inside this with 
            # block, the radius of each element of obstacle_list has been
            # expanded by 0.1 meters.
        # once we're out of the with block, obstacle_list will be
        # back to normal
    """
    for obs in obstacle_list:
        obs[2] += delta
    yield obstacle_list
    for obs in obstacle_list:
        obs[2] -= delta

class ConfigurationSpace(object):
    """ An abstract class for a Configuration Space. 
    """

    def __init__(self, dim, low_lims, high_lims, obstacles, dt=0.01):
        """
        Parameters
        ----------
        dim: dimension of the state space: number of state variables.
        low_lims: the lower bounds of the state variables. Should be an
                iterable of length dim.
        high_lims: the higher bounds of the state variables. Should be an
                iterable of length dim.
        obstacles: A list of obstacles. [x y z]
        dt: The discretization timestep our local planner should use when constructing
            plans.
        """
        self.dim = dim
        self.low_lims = np.array(low_lims)
        self.high_lims = np.array(high_lims)
        self.obstacles = obstacles
        self.dt = dt

    def distance(self, c1, c2):
        """
            Implements the chosen metric for this configuration space.
            This method should be implemented whenever this ConfigurationSpace
            is subclassed.

            Returns the distance between configurations c1 and c2 according to
            the chosen metric.
        """
        pass

    def sample_config(self, *args):
        """
            Samples a new configuration from this C-Space according to the
            chosen probability measure.
            This method should be implemented whenever this ConfigurationSpace
            is subclassed.

            Returns a new configuration sampled at random from the configuration
            space.
        """
        pass

    def check_collision(self, c):
        """
            Checks to see if the specified configuration c is in collision with
            any obstacles.
            This method should be implemented whenever this ConfigurationSpace
            is subclassed.
        """
        pass

    def check_path_collision(self, path):
        """
            Checks to see if a specified path through the configuration space is 
            in collision with any obstacles.
            This method should be implemented whenever this ConfigurationSpace
            is subclassed.
        """
        pass

    def local_plan(self, c1, c2):
        """
            Constructs a plan from configuration c1 to c2.

            This is the local planning step in RRT. This should be where you extend
            the trajectory of the robot a little bit starting from c1. This may not
            constitute finding a complete plan from c1 to c2. Remember that we only
            care about moving in some direction while respecting the kinemtics of
            the robot. You may perform this step by picking a number of motion
            primitives, and then returning the primitive that brings you closest
            to c2.
        """
        pass

    def nearest_config_to(self, config_list, config):
        """
            Finds the configuration from config_list that is closest to config.
        """
        return min(config_list, key=lambda c: self.distance(c, config))

class FishConfigurationSpace(ConfigurationSpace):
    """
        The configuration space for a Fish modeled robot
        Obstacles should be tuples (x, y, r), representing circles of 
        radius r centered at (x, y)
        The state of the robot is defined as (x, y, theta, phi).
    """
    def __init__(self, low_lims, high_lims, input_low_lims, input_high_lims, obstacles, robot_radius):
        dim = 3
        super(FishConfigurationSpace, self).__init__(dim, low_lims, high_lims, obstacles)
        self.input_low_lims = input_low_lims
        self.input_high_lims = input_high_lims
        self.robot_buffer = 0.2
    
    def distance(self, c1, c2):
        """
        c1 and c2 should be numpy.ndarrays of size (4,)
        """
        c1_x = c1[0]
        c1_y = c1[1]
        c2_x = c2[0]
        c2_y = c2[1]
        phi1 = c1[2]
        phi2 = c2[2]

        dist = np.sqrt((c1_x-c2_x)**2 + (c1_y - c2_y)**2 + (min(abs(phi1 - phi2), 2*np.pi - abs(phi1-phi2)) % 2*np.pi)**2)
        return dist
        
    def sample_config(self, *args):
        """
        Pick a random configuration from within our state boundaries.
        """
        random = np.random.rand()
        if random > 0.3:
            return np.array(args[0])
    
        return np.random.uniform(self.low_lims, self.high_lims).reshape((self.dim,))

    def check_collision(self, c):
        """
        Returns true if a configuration c is in collision
        c should be a numpy.ndarray of size (4,)
        """
        x = c[0]
        y = c[1]
        for obst in self.obstacles:
            # simulation
            # if (x - obst[0])**2 + (y - obst[1])**2 <= (obst[2]+self.robot_buffer)**2:
            #     return True
            # hardware
            if x >= obst[0] and x <= obst[2] and y >= obst[1] and y <= obst[3]:
                return True
        return False

    def check_path_collision(self, path):
        """
        Returns true if the input path is in collision. The path
        is given as a Plan object.
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
        """
        The fish bot resembles the unicycle dynamics model with slight modification as we focus on a point
        that we pivot around.
        Args:
            c1: starting point.
            c2: sampled point for local plan navigationl.
            dt (int): time step.
        Returns:
            best_plan: This should return a cofiguration_space.Plan object.
        """

        best_dist = np.inf
        best_plan = None

        for u1 in np.linspace(self.input_low_lims[0], self.input_high_lims[0], 17):
            for u2 in np.linspace(self.input_low_lims[1], self.input_high_lims[1], 10):
                total_time = 1
                c1_upd = c1
                positions = [np.array(c1)]
                velocities = len(np.arange(0, total_time, dt))*[[u1, u2]]
                times = np.arange(0, total_time, dt)
                for t in np.arange(dt, total_time, dt):
                    if u1 == 0:
                        q_dot = np.array([np.cos(c1_upd[2]) * u2, np.sin(c1_upd[2]) * u2, 0])
                    else:
                        q_dot = np.array([0, 0, (c1_upd[2] - u1)])
                    c1_upd = c1_upd + q_dot*dt
                    c1_upd[2] = c1_upd[2]  % (2*np.pi)
                    positions.append(c1_upd)
                plan = Plan(times, positions, velocities, dt=dt)
                if self.distance(plan.end_position(), c2) < best_dist:
                    best_dist = self.distance(plan.end_position(), c2)
                    best_plan = plan
        return best_plan
