from platform_class import Platform
from maestro import ServoController
from controller import PIDRegulator
from ball_data import BallData
import numpy as np
import time


def main():
    servo_controller = ServoController(np.array([1360, 1610, 1440, 1630, 1390, 1500]), 'COM6')
    platform = Platform(base_radius=(280.7 + 6)/2, base_distribution_angle=85, platform_radius=262.3/2,
                        platform_distribution_angle=15, height=270, servo_motor_arm_length=55, link_length=282,
                        thickness=0, platform_side_length=0.5)

    translation_vector = np.array([0, 0, 0])
    rotation_vector = np.array([0, 0, 0])
    platform.set_translation_and_rotation(translation_vector, rotation_vector)
    servo_controller.set_servo_motors_angles(platform.get_servo_motors_angles())

    pid = PIDRegulator(kp=25, ki=0, kd=20.588, ts=0.033, set_point=[0, 0], control_value_limit=5,
                       control_value_rate_of_change_limit=40)
    coords_finder = BallData(arduino_port='COM8')

    start = time.time()

    while True:
        coords = coords_finder.read_coords()
        trajectory = coords_finder.read_trajectory(0)
        # tryb: 1 -> okrąg, 2 -> kwadrat, 3 -> skok, 4 -> trajektoria własna, inne -> punkt

        while time.time() - start < 0.033:
            pass
        start = time.time()

        if coords is not None:
            pid.update_set_point(trajectory)
            theta = pid.update_PI_D([coords[0]/1000, coords[1]/1000])
            platform.set_translation_and_rotation(np.array([0, 0, 0]), np.array([0, theta[1], theta[0]]))
            servo_controller.set_servo_motors_angles(platform.get_servo_motors_angles())
            pid.write_data(20)
            coords_finder.write_data(20)


if __name__ == '__main__':
    main()
