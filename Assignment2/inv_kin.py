# -*- coding: utf-8 -*-
"""
Created on Mon Mar 20 12:01:49 2023

@author: Bijo Sebastian
Code completed by: AyushJam
"""

import math
import numpy as np
import robot_params


def inv_kin_fn(goal_position):
    # Compute joint angles [in degrees] to reach desired position [in meters]
    x_desired = goal_position[0]
    y_desired = goal_position[1]

    l1 = robot_params.link_1_length
    l2 = robot_params.link_2_length

    # Three DoF planar manipulator
    # Algebraic solution
    cos_theta_2 = (x_desired**2 + y_desired**2 - l1**2 - l2**2) / (2 * l1 * l2)

    if np.abs(cos_theta_2) > 1:
        # no solution
        raise ValueError("Desired point out of workspace: No Solution")
    else:
        # solutions possible
        sin_theta_2 = [
            np.sqrt(1 - cos_theta_2**2),
            -np.sqrt(1 - cos_theta_2**2),
        ]  # two solutions

        theta_2 = np.arctan2(
            sin_theta_2, cos_theta_2
        )  # arctan2 is a 4 quadrant inverse
        # in radian

        M = l1 + l2 * np.cos(theta_2)
        N = l2 * np.sin(theta_2)

        theta_1 = np.arctan2(y_desired, x_desired) - np.arctan2(N, M)

        theta_1 = [t * 180 / math.pi for t in theta_1]
        theta_2 = [t * 180 / math.pi for t in theta_2]

    # arbitrarily take the first solution set as only the end effector position is required
    print("Desired joint angles", [theta_1[0], theta_2[0]])
    return [theta_1[0], theta_2[0]]
