#!/usr/bin/env python
# -*- coding: utf-8 -*-



import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

show_animation = False


def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """

    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 0.8  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yawrate = 100.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 1  # [m/ss]
        self.max_dyawrate = 100.0 * math.pi / 180.0  # [rad/ss]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.v_reso = self.max_accel*self.dt/10.0  # [m/s]
        self.yawrate_reso = self.max_dyawrate*self.dt/10.0  # [rad/s]
        self.predict_time = 2  # [s]
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 0.1
        self.obstacle_cost_gain = 1.0
        self.robot_type = RobotType.rectangle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.4  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.3  # [m] for collision check
        self.robot_length = 0.6  # [m] for collision check

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


def motion(x, u, dt):
    """
    motion model
    """
    # x [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin, vmax, yaw_rate min, yaw_rate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    traj = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt

    return traj


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    to_goal_set, speed_set, ob_set = [], [], []
    trajectory_set, vy_set = [], []
    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):
            trajectory = predict_trajectory(x_init, v, y, config)

            # calc cost

            to_goal_set.append(calc_to_goal_cost(trajectory, goal))
            speed_set.append(config.max_speed - abs(np.mean(trajectory[:, 3])))
            ob_set.append(calc_obstacle_cost(trajectory, ob, config))

            trajectory_set.append(trajectory)
            vy_set.append([v, y])

    to_goal_set, speed_set, ob_set = np.array(to_goal_set), np.array(speed_set), np.array(ob_set)
    to_goal_set = to_goal_set / np.sum(to_goal_set)
    speed_set = speed_set / np.sum(speed_set)
    ob_set = ob_set / np.sum(ob_set)
    final_set = to_goal_set + 3*speed_set + 2*ob_set

    ind = np.argmin(final_set)
    temp = np.min(final_set)
    print(temp)

    best_u = vy_set[ind]
    best_trajectory = trajectory_set[ind]

    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
        calc obstacle cost inf: collision
    """
    cost = 0

    if ob.size == 0:
        return cost

    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    mind = np.min(r)
    radius = np.hypot(config.robot_length/2, config.robot_width/2)
    if mind < radius:
        cost = math.exp(-(mind-5))

    return cost  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """
    goal_angle = np.arctan2(goal[1], goal[0])

    if trajectory[-1, 3] >= 0:
        cost = math.fabs(goal_angle - trajectory[-1, 2])
    else:
        diff = math.fabs(goal_angle - (trajectory[-1, 2]+math.pi))
        if diff > math.pi:
            diff = 2*math.pi - diff
        cost = math.fabs(diff)

    cost += np.hypot(goal[0] - trajectory[-1, 0], goal[1] - trajectory[-1, 1])
    return cost
