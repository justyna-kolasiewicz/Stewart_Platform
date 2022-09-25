import calculator
import numpy as np


class Platform:
    def __init__(self, base_radius, base_distribution_angle, platform_radius, platform_distribution_angle, height,
                 servo_motor_arm_length, link_length, thickness, platform_side_length):
        self.__base_radius = base_radius
        self.__base_distribution_angle = base_distribution_angle
        self.__platform_radius = platform_radius
        self.__platform_distribution_angle = platform_distribution_angle
        self.__height = height
        self.__servo_motor_arm_length = servo_motor_arm_length
        self.__link_length = link_length
        self.__platform_joints = np.zeros((3, 6))
        self.__servo_motors_angles = np.zeros(6)
        self.__thickness = thickness
        self.__platform_side_length = platform_side_length
        self.__base_joints = calculator.get_base_geometry(self.__base_radius, self.__base_distribution_angle,
                                                          self.__height, self.__thickness)
        self.__initial_platform_joints = calculator.get_platform_geometry(self.__platform_radius,
                                                                          self.__platform_distribution_angle,
                                                                          self.__thickness)
        self.set_translation_and_rotation(np.array([0, 0, 0]), np.array([0, 0, 0]))

    def get_base_joints(self):
        return self.__base_joints

    def get_platform_joints(self):
        return self.__platform_joints

    def get_servo_motors_angles(self):
        return self.__servo_motors_angles

    def get_platform_side_length(self):
        return self.__platform_side_length

    def set_translation_and_rotation(self, translation_vector, rotation_vector):
        links_position = calculator.get_links_position(self.__base_joints, self.__initial_platform_joints,
                                                       translation_vector, rotation_vector)
        self.__platform_joints = np.add(self.__base_joints, links_position)
        links_length = calculator.get_links_length(links_position)
        self.__servo_motors_angles = calculator.get_servo_motors_angles(self.__base_joints, self.__platform_joints,
                                                                        links_length, self.__servo_motor_arm_length,
                                                                        self.__link_length,
                                                                        self.__base_distribution_angle)
