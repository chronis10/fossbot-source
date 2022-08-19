"""
Implementation of simulated control
"""

import math
import time
import threading
from common.interfaces import control_interfaces
from coppeliasim_robot import sim

def init_component(client_id: int, component_name: str) -> int:
    '''
    Initializes a component (like motors, sensors etc) of the simulation.
    Param: component_name: the name of the component (example: 'left_motor')
    Returns: the component in the simulation
    '''
    _, component = sim.simxGetObjectHandle(client_id, component_name, sim.simx_opmode_blocking)
    return component


def calc_distance_3d(x_dif: float, y_dif: float, z_dif: float) -> float:
    '''
    Calculates the distance of 2 points in 3d space
    Param: x_dif: the differnce between 2 points in x axis (x2-x1)
            y_dif: the differnce between 2 points in y axis (y2-y1)
            z_dif: the differnce between 2 points in z axis (z2-z1)
    Returns: the distance
    '''
    return math.sqrt(x_dif**2 + y_dif**2 + z_dif**2)


class AnalogueReadings(control_interfaces.AnalogueReadingsInterface):
    '''
    Analogue Readings
    AnalogueReadings(client_id)
    '''
    def __init__(self, client_id: int):
        self.floor_sensor_middle = init_component(client_id, "MiddleSensor")
        self.floor_sensor_left = init_component(client_id, "LeftSensor")
        self.floor_sensor_right = init_component(client_id, "RightSensor")
        self.client_id = client_id

    def __get_image(self, floor_sensor_name: str) -> list:
        _, floor_sensor = sim.simxGetObjectHandle(self.client_id, floor_sensor_name,
                                                  sim.simx_opmode_blocking)
        sim.simxGetVisionSensorImage(self.client_id, floor_sensor,
                                     0, sim.simx_opmode_streaming)
        time.sleep(0.1)
        _, _, image=sim.simxGetVisionSensorImage(self.client_id, floor_sensor,
                                                 0, sim.simx_opmode_buffer)
        return image

    def get_reading(self, pin: int) -> list:
        if pin == 1:
            return self.__get_image("MiddleSensor")
        elif pin == 2:
            return self.__get_image("RightSensor")
        elif pin == 3:
            return self.__get_image("LeftSensor")


class Motor(control_interfaces.MotorInterface):
    """
    Motor control
    Motor(client_id,motor_joint_name,def_speed)
    """

    def __init__(self, client_id: int, motor_joint_name: str, def_speed: int):
        self.client_id = client_id
        self.motor = init_component(self.client_id, motor_joint_name)
        self.motor_name = motor_joint_name
        self.def_speed = def_speed

    def __change_motor_velocity(self, motor: int, velocity: int) -> int:
        '''
        Changes a motor's velocity
        Param: motor: the motor of the simulation to change its velocity (example 'left_motor')
                velocity: the velocity to be changed to
        Returns: a return code of the API function
        '''
        return sim.simxSetJointTargetVelocity(self.client_id, motor,
                                              velocity, sim.simx_opmode_streaming)

    def dir_control(self, direction: str) -> None:
        """ Change motor direction """
        if direction == 'forward':
            self.__change_motor_velocity(self.motor, -self.def_speed)
        elif direction == "reverse":
            self.__change_motor_velocity(self.motor, self.def_speed)
        else:
            print("Motor accepts only forward and reverse values")

    def move(self, direction: str = "forward") -> None:
        """ Start motor to move with default speed """
        self.dir_control(direction)

    def set_speed(self, speed: int) -> None:
        """ Set speed immediately 0-100% range """
        if speed < 0 or speed > 100:
            print("The motor speed is a percentage of total motor power. Accepted values 0-100.")
        else:
            self.def_speed = self.def_speed * speed / 100
            self.__change_motor_velocity(self.motor, self.def_speed)

    def stop(self) -> None:
        """ Stops the motor"""
        self.__change_motor_velocity(self.motor, 0)


class Odometer(control_interfaces.OdometerInterface):
    '''
	Class Odometer() -> Odometer control
	Functions:
	count_revolutions() Increases the counter of revolutions
	get_revolutions() Returns the number of revolutions
	get_distance(wheel_diameter=6.6,precision = 2) Returns the traveled distance in cm
	reset() Resets the steps counter
	'''
    def __init__(self, client_id: int, motor_right: int):
        self.sensor_disc = 20   #by default 20 lines sensor disc
        self.steps = 0
        self.offset = 0
        self.wheel_diameter = 6.65  #by default the wheel diameter is 6.6
        self.precision = 2  #by default the distance is rounded in 2 digits
        self.client_id = client_id
        self.motor = motor_right
        sim.simxGetJointPosition(self.client_id, self.motor, sim.simx_opmode_streaming)
        self.run_thread = True
        self.step_thread = threading.Thread(target=self.__find_revolutions, daemon=True)
        self.step_thread.start()

    def __get_joint_pos(self) -> int:
        _, pos = sim.simxGetJointPosition(self.client_id, self.motor, sim.simx_opmode_buffer)
        return math.degrees(pos)

    def __check_rotations(self, start_pos: float) -> float:
        cur_pos = self.__get_joint_pos()
        #360/20=18
        dif = math.sqrt((cur_pos - start_pos)**2)
        steps, _ = divmod((dif / 18), 1)
        self.__increase_revolutions(steps)
        return cur_pos

    def __find_revolutions(self) -> None:
        start_pos = self.__get_joint_pos()
        while True:
            time.sleep(0.01)
            start_pos = self.__check_rotations(start_pos)

    def __increase_revolutions(self, steps: float) -> None:
        """ Increase total revolutions by steps  """
        self.steps += steps

    def count_revolutions(self) -> None:
        """ Increase total revolutions by one  """
        self.steps += 1

    def get_steps(self) -> int:
        """ Return total number of steps """
        return self.steps

    def get_revolutions(self) -> float:
        """ Return total number of revolutions """
        return self.steps / self.sensor_disc

    def get_distance(self) -> float:
        """ Return the total distance so far """
        self.run_thread = True
        circumference = self.wheel_diameter * math.pi
        revolutions = self.steps / self.sensor_disc
        distance = revolutions * circumference
        return (round(distance, self.precision)) + self.offset

    def reset(self) -> None:
        """ Reset the total distance and revolutions """
        self.steps = 0
        self.run_thread = False

class UltrasonicSensor(control_interfaces.UltrasonicSensorInterface):
    '''
	Class UltrasonicSensor() -> Ultrasonic sensor
	Functions:
	get_distance() return distance on cm
    '''
    def __init__(self, client_id: int):
        self.client_id = client_id
        self.ultrasonic = init_component(self.client_id, "ultrasonic_sensor")
        self.__get_near_obst(mode=sim.simx_opmode_streaming)  # 1st call

    def __get_near_obst(self, mode: int = sim.simx_opmode_buffer) -> list:
        _, _, detected_point, _, _ = sim.simxReadProximitySensor(self.client_id,
                                                                 self.ultrasonic, mode)
        return detected_point

    def get_distance(self) -> float:
        detected_point = self.__get_near_obst(mode=sim.simx_opmode_buffer)
        print(detected_point)
        return calc_distance_3d(detected_point[0], detected_point[1], detected_point[2])


class Accelerometer(control_interfaces.AccelerometerInterface):
    '''
    Class accelerometer(address=0x68)
    Functions:
    get_acceleration(dimension = "all") with "parameter return dictionary with x,y,z
                                        acceleration for a specific dimension give
                                        as parameter "x","y","z" return value
    get_gyro(dimension = "all") with "parameter return dictionary with x,y,z
                                acceleration for a specific dimension give
                                as parameter "x","y","z" return value
    '''
    def __init__(self, client_id: int):
        self.sensor = init_component(client_id, 'sensor')
        sim.simxGetObjectVelocity(client_id, self.sensor, sim.simx_opmode_streaming)
        self.client_id = client_id

    def __get_linear_vel(self) -> float:
        _, linear_vel, _ = sim.simxGetObjectVelocity(self.client_id, self.sensor,
                                                     sim.simx_opmode_buffer)
        return linear_vel

    def __get_angular_vel(self) -> float:
        _, _, angular_vel = sim.simxGetObjectVelocity(self.client_id, self.sensor,
                                                      sim.simx_opmode_buffer)
        return angular_vel

    def __get_accel_data(self, time_dif: int = 1) -> dict:
        v_1 = self.__get_linear_vel()
        time.sleep(time_dif)
        v_2 = self.__get_linear_vel()
        x_vel = ((v_2[0] - v_1[0]) / time_dif)
        y_vel = ((v_2[1] - v_1[1]) / time_dif)
        z_vel = ((v_2[2] - v_1[2]) / time_dif)
        return {'x':x_vel, 'y':y_vel, 'z':z_vel}

    def __get_gyro_data(self, time_dif: int = 1) -> dict:
        v_1 = self.__get_angular_vel()
        time.sleep(time_dif)
        v_2 = self.__get_angular_vel()
        x_vel = ((v_2[0] - v_1[0]) / time_dif)
        y_vel = ((v_2[1] - v_1[1]) / time_dif)
        z_vel = ((v_2[2] - v_1[2]) / time_dif)
        return {'x':x_vel, 'y':y_vel, 'z':z_vel}

    def get_acceleration(self, dimension: str = "all") -> dict:
        accel = self.__get_accel_data()
        if dimension == "all":
            return accel
        if dimension in ('x', 'y', 'z'):
            return accel[dimension]
        print("Dimension not recognized!!")
        return 0

    def get_gyro(self, dimension: str = "all") -> dict:
        gyro = self.__get_gyro_data()
        if dimension == "all":
            return gyro
        if dimension in ('x', 'y', 'z'):
            return gyro[dimension]
        print("Dimension not recognized!!")
        return 0
