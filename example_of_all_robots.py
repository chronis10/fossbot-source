""" Example of a real and simulated robot"""

import time
from parameters_parser.parser import load_parameters
from common.data_structures import configuration
from common.interfaces import robot_interface
from real_robot.fossbot import FossBot as RealFossBot
from coppeliasim_robot.fossbot import FossBot as SimuFossBot

def main(robot: robot_interface.FossBotInterface) -> None:
    """ A simple robot routine """
    robot.just_move()
    time.sleep(10)
    robot.stop()

if __name__ == "__main__":
    # Load parameters from yml file
    FILE_PARAM = load_parameters()

    # Create dataclass using file parameters
    parameters = configuration.RobotParameters(
        sensor_distance=configuration.SensorDistance(**FILE_PARAM["sensor_distance"]),
        motor_left_speed=configuration.MotorLeftSpeed(**FILE_PARAM["motor_left"]),
        motor_right_speed=configuration.MotorRightSpeed(**FILE_PARAM["motor_right"]),
        default_step=configuration.DefaultStep(**FILE_PARAM["step"]),
        light_sensor=configuration.LightSensor(**FILE_PARAM["light_sensor"]),
        line_sensor_left=configuration.LineSensorLeft(**FILE_PARAM["line_sensor_left"]),
        line_sensor_center=configuration.LineSensorCenter(**FILE_PARAM["line_sensor_center"]),
        line_sensor_right=configuration.LineSensorRight(**FILE_PARAM["line_sensor_right"]),
        rotate_90=configuration.Rotate90(**FILE_PARAM["rotate_90"]))

    # Create a real robot
    my_real_robot = RealFossBot(parameters=parameters)

    # Create a simu robot
    my_simu_robot = SimuFossBot(parameters=parameters)

    # Call a the same routine for the real and the simulated robot
    main(my_real_robot)

    main(my_simu_robot)
