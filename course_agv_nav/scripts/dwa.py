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
    min_vcost = float("inf")
    min_obcost = float("inf")
    min_gcost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # plt.figure()
    # plt.plot(goal[0], goal[1],'ro')
    # plt.grid(True)

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):
            # print("v,y: ", v, y)
            trajectory = predict_trajectory(x_init, v, y, config)

            # calc cost

            # TODO here

            # final_cost = to_goal_cost + speed_cost + ob_cost

            final_cost = calc_to_goal_cost(trajectory, goal) + calc_obstacle_cost(trajectory, ob, config)

            v_mean = np.mean(trajectory[:,3])
            if v_mean >= 0:
                v_cost = (config.max_speed - v_mean)/config.max_speed
            else:
                v_cost = (-config.min_speed + v_mean)/(-config.min_speed)

            v_cost = 0.8 - math.fabs(v_mean)

            # v_cost = v_mean

            # print("vcost: ",v_cost)

            final_cost += v_cost*3
            # plt.plot(trajectory[:,0], trajectory[:,1], 'g')

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_vcost = v_cost
                min_gcost = calc_to_goal_cost(trajectory, goal)
                min_obcost = calc_obstacle_cost(trajectory, ob, config)

                best_u = [v, y]
                best_trajectory = trajectory
                print('min_gcost',min_gcost)
                print('min_vcost', min_vcost)
                print('min_obcost', min_obcost) 
    #             print("plot one")




    print('min_gcost',min_gcost)
    print('min_vcost', min_vcost)
    print('min_obcost', min_obcost)       

    # plt.plot(best_trajectory[:,0], best_trajectory[:,1], 'b')
    # plt.show()

    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
        calc obstacle cost inf: collision
    """
    if ob.size == 0:
        cost = 0
        return cost


    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    cost = 0
    # TODO here

    mind = r.min()
    # print("rmin:",mind)
    if mind < 0.4:
        cost = math.exp(-(mind-5))

    return cost  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    # TODO here
    goal_angle = math.atan2(goal[1], goal[0])
    # print("goal: ", goal[0], goal[1])
    # print("goal_angle: ",goal_angle)

    if trajectory[-1,3] >= 0:
        cost = math.fabs(goal_angle - trajectory[-1, 2])
    else:
        diff = math.fabs(goal_angle - (trajectory[-1, 2]+math.pi))
        if diff > math.pi:
            diff = 2*math.pi - diff
        cost = math.fabs(diff)

    # print("goal_cost:", cost)

    return cost
