import numpy as np


def get_base_geometry(base_radius, base_distribution_angle, height, thickness):
    alpha = base_distribution_angle / 2
    thetas_in_deg = np.array([90 - alpha, 90 + alpha, 210 - alpha, 210 + alpha, 330 - alpha, 330 + alpha])
    thetas_in_rad = np.radians(thetas_in_deg)
    base_joints = np.zeros((3, 6))

    for num, theta in enumerate(thetas_in_rad):
        base_joints[0, num] = base_radius * np.cos(theta)
        base_joints[1, num] = base_radius * np.sin(theta)
        base_joints[2, num] = -height - thickness

    return base_joints


def get_platform_geometry(platform_radius, platform_distribution_angle, thickness):
    alpha = platform_distribution_angle / 2
    thetas_in_deg = np.array([30 + alpha, 150 - alpha, 150 + alpha, 270 - alpha, 270 + alpha, 30 - alpha])
    thetas_in_rad = np.radians(thetas_in_deg)
    platform_joints = np.zeros((3, 6))

    for num, theta in enumerate(thetas_in_rad):
        platform_joints[0, num] = platform_radius * np.cos(theta)
        platform_joints[1, num] = platform_radius * np.sin(theta)
        platform_joints[2, num] = -thickness

    return platform_joints


def get_links_position(base_joints, platform_joints, translation_vector, rotation_vector):
    a, b, c = np.radians(rotation_vector)
    translation_matrix = np.tile(translation_vector.reshape(3, 1), (1, 6))
    rotation_matrix = np.array([
        [np.cos(a) * np.cos(b), np.cos(a) * np.sin(b) * np.sin(c) - np.sin(a) * np.cos(c),
         np.cos(a) * np.sin(b) * np.cos(c) + np.sin(a) * np.sin(c)],
        [np.sin(a) * np.cos(b), np.sin(a) * np.sin(b) * np.sin(c) + np.cos(a) * np.cos(c),
         np.sin(a) * np.sin(b) * np.cos(c) - np.cos(a) * np.sin(c)],
        [-np.sin(b), np.cos(b) * np.sin(c), np.cos(b) * np.cos(c)]
    ])

    links_position = np.subtract(np.add(translation_matrix, rotation_matrix.dot(platform_joints)), base_joints)
    return links_position


def get_links_length(links_position):
    links_length = np.zeros(6)

    for num, coordinates in enumerate(links_position.transpose()):
        links_length[num] = np.sqrt(
            np.power(coordinates[0], 2) + np.power(coordinates[1], 2) + np.power(coordinates[2], 2))

    return links_length


def get_servo_motors_angles(base_joints, platform_joints, links_length, servo_motor_arm_length, link_length,
                            base_distribution_angle):
    servo_motors_angles = np.zeros(6)
    alpha = base_distribution_angle / 2
    thetas_in_deg = np.array([90 - alpha, 90 + alpha, 210 - alpha, 210 + alpha, 330 - alpha, 330 + alpha])
    thetas_in_rad = np.radians(thetas_in_deg)
    for num, (theta, length) in enumerate(zip(thetas_in_rad, links_length)):
        a = 2 * servo_motor_arm_length * (platform_joints[2, num] - base_joints[2, num])
        b = 2 * servo_motor_arm_length * (
                    np.sin(theta) * (platform_joints[0, num] - base_joints[0, num]) - np.cos(theta) * (
                        platform_joints[1, num] - base_joints[1, num]))
        c = np.power(abs(length), 2) - np.power(link_length, 2) + np.power(servo_motor_arm_length, 2)
        if num % 2:
            servo_motors_angles[num] = np.arcsin(c / np.sqrt(a * a + b * b)) - np.arctan(b / a)
        else:
            servo_motors_angles[num] = np.arcsin(-c / np.sqrt(a * a + b * b)) - np.arctan(b / a)
    servo_motors_angles = np.degrees(servo_motors_angles)
    return servo_motors_angles
