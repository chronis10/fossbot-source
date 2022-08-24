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


def get_object_children(client_id: int, object_name: str = '/', print_all = False) -> tuple:
    '''
    Retrieves handles of all the children of an object
    Default object_name: '/': retrieves all the objects handles in the scene
    Recommended object_name: 'fossbot': retrieves all children of fossbot 
    Param: client_id: the client id
           object_name: the object's name in the scene
           print_all: prints all the handles and their corresponding object's path in the scene
    Returns: object_children_list: a list of all the childrens handles of the requested object
             object_children_dict: a dictionary with keys the handles and values the 
                                  corresponding path in the scene of the requested object
    '''
    sim.simxGetObjectGroupData(client_id, sim.sim_appobj_object_type, 21, sim.simx_opmode_streaming)
    time.sleep(0.1)
    _, handle, _, _, name = sim.simxGetObjectGroupData(client_id, sim.sim_appobj_object_type, 21, sim.simx_opmode_blocking)

    object_children_list=[]
    object_children_dict={}

    if not object_name.startswith('/'):
        object_name = '/' + object_name

    for h in handle:
        if print_all:
            print(f'Handle: {h}, Path: {name[h]}')
        if object_name in name[h]:
            object_children_list.append(h)
            object_children_dict[h] = name[h]

    if len(object_children_list) == 0:
        print(f'There is no robot named {object_name[1:]} in scene.')
        raise ModuleNotFoundError

    return object_children_list, object_children_dict


class AnalogueReadings(control_interfaces.AnalogueReadingsInterface):
    '''
    Analogue Readings
    AnalogueReadings(client_id)
    '''
    def __init__(self, client_id: int):
        self.client_id = client_id
        self.floor_sensor_middle = init_component(client_id, "MiddleSensor")
        self.floor_sensor_left = init_component(client_id, "LeftSensor")
        self.floor_sensor_right = init_component(client_id, "RightSensor")
        # add sleep -> brings more results
        sim.simxGetVisionSensorImage(self.client_id, self.floor_sensor_middle, 0, sim.simx_opmode_streaming)
        time.sleep(0.1)
        sim.simxGetVisionSensorImage(self.client_id, self.floor_sensor_right, 0, sim.simx_opmode_streaming)
        time.sleep(0.1)
        sim.simxGetVisionSensorImage(self.client_id, self.floor_sensor_left, 0, sim.simx_opmode_streaming)
        time.sleep(0.1)

    def get_reading(self, pin: int) -> list:
        # add sleep -> brings more results
        if pin == 1:
            time.sleep(0.1)
            _, _, image=sim.simxGetVisionSensorImage(self.client_id, self.floor_sensor_middle,
                                                     0, sim.simx_opmode_buffer)
            return image
        elif pin == 2:
            time.sleep(0.1)
            _, _, image=sim.simxGetVisionSensorImage(self.client_id, self.floor_sensor_right,
                                                     0, sim.simx_opmode_buffer)
            return image
        elif pin == 3:
            time.sleep(0.1)
            _, _, image=sim.simxGetVisionSensorImage(self.client_id, self.floor_sensor_left,
                                                     0, sim.simx_opmode_buffer)
            return image

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
        '''
        Change motor direction
        Param: direction: the direction to be headed to
        '''
        if direction == 'forward':
            self.__change_motor_velocity(self.motor, -self.def_speed)
        elif direction == "reverse":
            self.__change_motor_velocity(self.motor, self.def_speed)
        else:
            print("Motor accepts only forward and reverse values")

    def move(self, direction: str = "forward") -> None:
        '''
        Start moving motor with default speed
        Param: direction: the direction to be headed to
        '''
        self.dir_control(direction)

    def set_speed(self, speed: int) -> None:
        '''
        Set speed immediately 0-100% range
        Param: speed: the range 0 - 100% that speed will be changed to
        '''
        if speed < 0 or speed > 100:
            print("The motor speed is a percentage of total motor power. Accepted values 0-100.")
        else:
            self.def_speed = self.def_speed * speed / 100
            self.__change_motor_velocity(self.motor, self.def_speed)

    def stop(self) -> None:
        '''Stops the motor'''
        self.__change_motor_velocity(self.motor, 0)


class Odometer(control_interfaces.OdometerInterface):
    '''
	Class Odometer() -> Odometer control
	Functions:
	count_revolutions() Increases the counter of revolutions
	get_revolutions() Returns the number of revolutions
	get_distance() Returns the traveled distance in cm
	reset() Resets the steps counter
	'''
    def __init__(self, client_id: int, motor_right: int):
        self.sensor_disc = 20   #by default 20 lines sensor disc
        self.steps = 0
        self.wheel_diameter = 6.65  #by default the wheel diameter is 6.6
        self.precision = 2  #by default the distance is rounded in 2 digits
        self.client_id = client_id
        self.motor = motor_right
        sim.simxGetJointPosition(self.client_id, self.motor, sim.simx_opmode_streaming)
        self.step_thread = threading.Thread(target=self.__find_revolutions, daemon=True)
        self.step_thread.start()

    def __get_joint_pos(self) -> int:
        '''
        Retrieves the anglar position of a motor
        Returns: the angular position of a motor in deegres
        '''
        _, pos = sim.simxGetJointPosition(self.client_id, self.motor, sim.simx_opmode_buffer)
        return math.degrees(pos)

    def __check_rotations(self, start_pos: float) -> float:
        '''
        Checks and increases steps
        Param: start_pos: the starting angle of the motor
        Returns: cur_pos: the current angle of the motor
        '''
        cur_pos = self.__get_joint_pos()
        #360/20=18
        dif = math.sqrt((cur_pos - start_pos)**2)
        steps, _ = divmod((dif / 18), 1)
        self.__increase_revolutions(steps)
        return cur_pos

    def __find_revolutions(self) -> None:
        '''
        Runs in the backround and calculates revolutions
        '''
        start_pos = self.__get_joint_pos()
        while True:
            time.sleep(0.01)
            start_pos = self.__check_rotations(start_pos)

    def __increase_revolutions(self, steps: float) -> None:
        '''
        Increase total steps by n
        Param: steps: the number n that steps will be increased by 
        '''
        self.steps += steps

    def count_revolutions(self) -> None:
        ''' Increase total steps by one '''
        self.steps += 1

    def get_steps(self) -> int:
        ''' Returns total number of steps '''
        return self.steps

    def get_revolutions(self) -> float:
        ''' Return total number of revolutions '''
        return self.steps / self.sensor_disc

    def get_distance(self) -> float:
        ''' Return the total distance so far (in cm) '''
        circumference = self.wheel_diameter * math.pi
        revolutions = self.steps / self.sensor_disc
        distance = revolutions * circumference
        return (round(distance, self.precision))

    def reset(self) -> None:
        ''' Reset the total distance and revolutions '''
        self.steps = 0

class UltrasonicSensor(control_interfaces.UltrasonicSensorInterface):
    '''
	Class UltrasonicSensor() -> Ultrasonic sensor
	Functions:
	get_distance() return distance in cm
    '''
    def __init__(self, client_id: int):
        self.client_id = client_id
        self.precision = 2  #by default the distance is rounded in 2 digits

    def get_distance(self) -> float:
        '''
        Gets the distance to the closest obstacle
        Returns: the distance to the closest obstacle (in cm)
        If no obstacle detected => returns 999.9
        '''
        while True:
            res_1, handle, distance,_ ,_ = sim.simxCallScriptFunction(self.client_id, 'ultrasonic_sensor', sim.sim_scripttype_childscript, 'get_distance', [], [], [], bytearray(), sim.simx_opmode_blocking)
            if res_1 == sim.simx_return_ok and handle[0] == sim.simx_return_ok:
                break
        #Detected Handle: handle[1], Distance (in meters): distance[0]
        return (round(distance[0]*100, self.precision))

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
        self.client_id = client_id

    def __create_force_dict(self, force_list: list) -> dict:
        return {'x': force_list[0], 'y': force_list[1], 'z': force_list[2]}

    def get_acceleration(self, dimension: str = "all") -> dict:
        while True:
            # 1st response -> function executed correctly, 2ns response -> data was successfully collected
            res_1, res_2, accel_data,_ ,_ = sim.simxCallScriptFunction(self.client_id, 'Accelerometer', sim.sim_scripttype_childscript, 'get_accel', [], [], [], bytearray(), sim.simx_opmode_blocking)
            if res_1 == sim.simx_return_ok and res_2[0] == sim.simx_return_ok:
                break
        accel_data = self.__create_force_dict(accel_data)
        if dimension == "all":
            return accel_data
        if dimension in ('x', 'y', 'z'):
            return accel_data[dimension]
        print("Dimension not recognized!!")
        raise RuntimeError


    def get_gyro(self, dimension: str = "all") -> dict:
        while True:
            res, _, gyro_data,_ ,_ = sim.simxCallScriptFunction(self.client_id, 'GyroSensor', sim.sim_scripttype_childscript, 'get_gyro', [], [], [], bytearray(), sim.simx_opmode_blocking)
            if res == sim.simx_return_ok:
                break
        gyro_data = self.__create_force_dict(gyro_data)
        if dimension == "all":
            return gyro_data
        if dimension in ('x', 'y', 'z'):
            return gyro_data[dimension]
        print("Dimension not recognized!!")
        raise RuntimeError



class Led_RGB(control_interfaces.LedRGBInterface):
    def __init__(self, client_id: int):
        self.client_id = client_id

    def set_on(self, color: str) -> None:
        """ Changes the color of a led """
        color_rbg = [0, 0, 0]   #red, blue, green
        if color == 'red':
            color_rbg = [1, 0, 0]
        elif color == 'green':
            color_rbg = [0, 1, 0]
        elif color == 'blue':
            color_rbg = [0, 0, 1]
        elif color == 'white':
            color_rbg = [1, 1, 1]
        elif color == 'violet':
            color_rbg = [1, 1, 0]
        elif color == 'cyan':
            color_rbg = [0, 1, 1]
        elif color == 'yellow':
            color_rbg = [1, 0, 1]
        elif color == 'closed':
            color_rbg = [0, 0, 0]
        else:
            print('Uknown color!')
            raise RuntimeError

        sim.simxCallScriptFunction(self.client_id, 'led_light', sim.sim_scripttype_childscript, 'set_color_led', [], color_rbg, [], bytearray(), sim.simx_opmode_blocking)
