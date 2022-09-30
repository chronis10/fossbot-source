""" Example of a real and simulated robot"""
import time
import os
import pathlib
#from coppeliasim_robot import control
from fossbot_lib.parameters_parser.parser import load_parameters
from fossbot_lib.common.data_structures import configuration
from fossbot_lib.common.interfaces import robot_interface
from fossbot_lib.coppeliasim_robot.sim_gym import Environment
from fossbot_lib.coppeliasim_robot.fossbot import FossBot as SimuFossBot

def main(robot: robot_interface.FossBotInterface) -> None:
    """ A simple robot routine """
    robot.just_move()
    time.sleep(10)
    robot.stop()

def ultimate_test(robot: robot_interface.FossBotInterface) -> None:
    '''Tests important functions of robot'''
    robot.move_distance(10)
    robot.wait(1)
    robot.motor_left.set_speed(50)
    robot.motor_right.set_speed(50)
    robot.move_distance(11)
    robot.wait(1)
    robot.rotate_clockwise_90()
    robot.rotate_counterclockwise_90()
    robot.wait(1)
    print(f'Light Sensor: {robot.get_light_sensor()}')
    print(f'Dark: {robot.check_for_dark()}')
    print(f'Ultrasonic distance: {robot.get_distance()}')
    print(f'Obstacle => {robot.check_for_obstacle()}')
    print(f"Gyroscope x: {robot.get_gyroscope(axis='x')}")
    print(f"Acceleration x: {robot.get_acceleration(axis='x')}")
    print(f'MiddleSensor: {robot.get_floor_sensor(1)}, On Line? => {robot.check_on_line(1)}')
    print(f'RightSensor: {robot.get_floor_sensor(2)}, On Line? => {robot.check_on_line(2)}')
    print(f'LeftSensor: {robot.get_floor_sensor(3)}, On Line? => {robot.check_on_line(3)}')
    print('Changing led colors...')
    change_color(robot)

def follow_line(robot: robot_interface.FossBotInterface) -> None:
    '''Follows black line.'''
    while True:
        middle = robot.check_on_line(1)
        right = robot.check_on_line(2)
        left = robot.check_on_line(3)
        if middle:
            robot.move_forward()
        elif right:
            robot.rotate_counterclockwise()
        elif left:
            robot.rotate_clockwise()

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
    print('Closing led...')
    robot.rgb_set_color('closed')

def change_path_test(robot: robot_interface.FossBotInterface, environment: Environment) -> None:
    '''
    "Draws" images-paths on the floor.
    '''
    current_path = pathlib.Path(__file__).parent.resolve()
    path_dir_b = os.path.join(current_path, 'scenes')
    path_dir = os.path.join(path_dir_b, 'paths')
    print('Changing Path...')
    file_path = os.path.join(path_dir, 'Path1.jpg')
    environment.draw_path_auto(robot, file_path)
    time.sleep(3)
    print('Changing Path...')
    file_path = os.path.join(path_dir, 'Path2.jpg')
    environment.draw_path_auto(robot, file_path)
    time.sleep(3)
    print('Changing Path...')
    file_path = os.path.join(path_dir, 'Path3.jpg')
    environment.draw_path_auto(robot, file_path)
    time.sleep(3)
    print('Clearing Path...')
    environment.clear_path(robot)

def check_collision_test(robot: robot_interface.FossBotInterface) -> None:
    '''
    Prints True if fossbot has collided with collidable object.
    '''
    # displays all handles and names of objects in the scene:
    #control.get_object_children(SIM_IDS.client_id, print_all=True)
    while True:
        c_check = robot.check_collision()
        if c_check:
            print(c_check)

def inbounds_teleport_test(
        robot: robot_interface.FossBotInterface,
        environment: Environment) -> None:
    '''
    Tests teleportation with big values (so the robot will stay in bounds).
    Reminder:
        abs(max_x) = 2.3, abs(max_y) = 2.3
        (aka almost the half of the scale of floor), z==height.
        floor_scale: x==5, y==5.
    '''
    environment.teleport(robot, 100, 100, in_bounds=True)
    time.sleep(2)
    environment.teleport(robot, -100, 100, in_bounds=True)
    time.sleep(2)
    environment.teleport(robot, 100, -100, in_bounds=True)
    time.sleep(2)
    environment.teleport(robot, -100, -100, in_bounds=True)
    time.sleep(2)

def change_brightness_test(
        robot: robot_interface.FossBotInterface,
        environment: Environment) -> None:
    '''Test for changing brightness.'''
    environment.change_brightness(robot, 0)
    time.sleep(2)
    environment.change_brightness(robot, 100)
    time.sleep(2)
    environment.change_brightness(robot, 50)
    time.sleep(2)
    environment.change_brightness(robot, 70)
    time.sleep(2)

def ultimate_environment_test(
        environment: Environment,
        robot: robot_interface.FossBotInterface) -> None:
    '''Tests all environment functions.'''
    inbounds_teleport_test(robot, environment)
    environment.teleport_empty_space(robot)
    environment.clear_path(robot)
    change_path_test(robot, environment)
    change_brightness_test(robot, environment)
    environment.default_brightness(robot)
    environment.teleport_random(robot, in_bounds=False)
    current_path = pathlib.Path(__file__).parent.resolve()
    path_dir_b = os.path.join(current_path, 'scenes')
    path_dir = os.path.join(path_dir_b, 'paths')
    file_path = os.path.join(path_dir, 'Path1.jpg')
    environment.draw_path(robot, file_path, scale_x=3)

def detect_noise_test(robot: robot_interface.FossBotInterface, for_time: float = 0.5) -> None:
    '''
    Noise detection test for specific amount of time.
    Param: for_time: the time for the test to last.
    '''
    tar_time = time.time() + for_time
    while time.time() < tar_time:
        print(robot.get_noise_detection())

if __name__ == "__main__":
    # Load parameters from yml file
    FILE_PARAM = load_parameters()

    # Simulation robot test ===========================================
    SIM_IDS = configuration.SimRobotIds(**FILE_PARAM["simulator_ids"])
    SIM_PARAM = configuration.SimRobotParameters(
        sensor_distance=configuration.SensorDistance(**FILE_PARAM["sensor_distance"]),
        motor_left_speed=configuration.MotorLeftSpeed(**FILE_PARAM["motor_left"]),
        motor_right_speed=configuration.MotorRightSpeed(**FILE_PARAM["motor_right"]),
        default_step=configuration.DefaultStep(**FILE_PARAM["step"]),
        light_sensor=configuration.LightSensor(**FILE_PARAM["light_sensor"]),
        line_sensor_left=configuration.LineSensorLeft(**FILE_PARAM["line_sensor_left"]),
        line_sensor_center=configuration.LineSensorCenter(**FILE_PARAM["line_sensor_center"]),
        line_sensor_right=configuration.LineSensorRight(**FILE_PARAM["line_sensor_right"]),
        rotate_90=configuration.Rotate90(**FILE_PARAM["rotate_90"]),
        simulation=SIM_IDS)

    # Create a simu robot
    SIM_ROBOT = SimuFossBot(parameters=SIM_PARAM)
    # Create environment
    ENVIRONMENT = Environment()
    #main(SIM_ROBOT)
    #ultimate_test(SIM_ROBOT)
    #change_color(SIM_ROBOT)
    #follow_line(SIM_ROBOT)
    #check_collision_test(SIM_ROBOT)
    #detect_noise_test(SIM_ROBOT)

    # Environment Testing:
    ultimate_environment_test(ENVIRONMENT, SIM_ROBOT)
