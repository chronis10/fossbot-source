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

def ultimate_test(robot: robot_interface.FossBotInterface) -> None:
    '''Tests important functions of robot'''
    robot.move_distance(21)
    robot.wait(1)
    robot.rotate_clockwise_90()
    robot.rotate_counterclockwise_90()
    robot.wait(1)
    print(f'Light Sensor: {robot.get_light_sensor()}')
    print(f'Dark: {robot.check_for_dark()}')
    print(f'Ultrasonic distance: {robot.get_distance()}')
    print(f'Obstacle => {robot.check_for_obstacle()}')
    print(f'Gyroscope: {robot.get_gyroscope()}')
    print(f'Acceleration: {robot.get_acceleration()}')
    print(f'MiddleSensor: {robot.get_floor_sensor(1)}, On Line? => {robot.check_on_line(1)}')
    print(f'RightSensor: {robot.get_floor_sensor(2)}, On Line? => {robot.check_on_line(2)}')
    print(f'LeftSensor: {robot.get_floor_sensor(3)}, On Line? => {robot.check_on_line(3)}')
    print('Changing led colors...')
    change_color(robot)


def change_color(robot: robot_interface.FossBotInterface) -> None:
    ''' Changes the color of a led for some times '''
    for i in range(3):
        robot.rgb_set_color('red')
        robot.wait(2)
        robot.rgb_set_color('blue')
        robot.wait(2)
        robot.rgb_set_color('green')
        robot.wait(2)
        robot.rgb_set_color('yellow')
        robot.wait(2)
        robot.rgb_set_color('white')
        robot.wait(2)
        robot.rgb_set_color('cyan')
        robot.wait(2)
        robot.rgb_set_color('violet')
        robot.wait(2)
    robot.rgb_set_color('closed')

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
    #ultimate_test(my_simu_robot)
    #change_color(my_simu_robot)
