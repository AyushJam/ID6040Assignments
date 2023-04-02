# -*- coding: utf-8 -*-
"""
Created on Tue Mar 28 09:51:21 2023

@author: Bijo Sebastian
Code completed by: Ayush Jamdar
"""
import numpy as np
import math
import robot_params


def get_desired_joint_rate(joint_angles):

    l1 = robot_params.link_1_length
    l2 = robot_params.link_2_length

    # Compute 2x2 jacobian matrix for given joint angles [in radians]
    j11 = -l1 * math.sin(joint_angles[0]) - l2 * math.sin(
        joint_angles[0] + joint_angles[1]
    )
    j12 = -l1 * math.sin(joint_angles[0] + joint_angles[1])
    j21 = l1 * math.cos(joint_angles[0]) + l2 * math.cos(
        joint_angles[0] + joint_angles[1]
    )
    j22 = l2 * math.cos(joint_angles[0] + joint_angles[1])

    jacobian = np.array([[j11, j12], [j21, j22]])

    # raise an exception if jacobian is singular
    if np.linalg.det(jacobian) == 0:
        raise Exception("Jacobian is singular")
    else:
        # Compute desired joint angle rate by inverting jacobian and multiplying with desired end effector velocity
        desired_end_effector_velocity = np.array([[0.01], [0.0]])
        desired_joint_angle_rates = np.matmul(
            np.linalg.inv(jacobian), desired_end_effector_velocity
        )

    return [desired_joint_angle_rates[0], desired_joint_angle_rates[1]]
