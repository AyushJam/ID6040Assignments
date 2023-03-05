import math
import robot_params

l1 = robot_params.link_1_length  # file robot_params for link lengths
l2 = robot_params.link_2_length


def fw_kin(joint_angles):
    # Compute end effector position [in meters] for the given pair of joint angles [in degrees]

    # convert angles to radians (Note that 3.14 is used instead of math.pi)
    joint_angles_rad = [angle * (3.14 / 180) for angle in joint_angles]

    # compute end effector position
    x_end_effector = l1 * math.cos(joint_angles_rad[0]) + l2 * math.cos(
        joint_angles_rad[0] + joint_angles_rad[1]
    )
    y_end_effector = l1 * math.sin(joint_angles_rad[0]) + l2 * math.sin(
        joint_angles_rad[0] + joint_angles_rad[1]
    )

    end_effector_position_analytic = [x_end_effector, y_end_effector]

    print("End effector position (analytic)", end_effector_position_analytic)
    return end_effector_position_analytic
