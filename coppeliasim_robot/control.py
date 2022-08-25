"""
Implementation of simulated control
"""

import math
import time
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


def exec_vrep_script(client_id: int, script_component_name: str, script_function_name: str,
                     inInts: list = [], inFloats: list = [], inStrings: list = [],
                     inBuffer: bytearray = bytearray()) -> tuple:
    '''
    Executes a function of a lua script in vrep
    Param: client_id: the client's id
           script_component_name: the name of the object that has the script in the scene
           script_function_name: the name of the function inside the script to be executed
           inInts: list of input integers used for the function (can be [ ])
           inFloats: list of input floats used for the function (can be [ ])
           inStrings: list of input strings used for the function (can be [ ])
           inBuffer: input bytearray used for the function
    Returns: returnCode: to show if function has been executed correctly
             => (successful execution: sim.simx_return_ok)
             outInts: list of integer values returned by the function
             outFloats: list of float values returned by the function
             outStrings: list of string values returned by the function
             outBuffer: bytearray returned by the function
    '''
    return sim.simxCallScriptFunction(client_id, script_component_name, sim.sim_scripttype_childscript,
                                      script_function_name, inInts, inFloats, inStrings, inBuffer,
                                      sim.simx_opmode_blocking)


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

    def __get_line_data(self, line_sensor_name: str) -> list:
        '''
        Retrieves image data of requested line sensor
        Param: line_sensor_name: the name of the wanted line sensor
        Returns: image data of requested line_sensor
        '''
        while True:
            res, image, _,_ ,_ = exec_vrep_script(self.client_id, line_sensor_name, 'get_line_image')
            if res == sim.simx_return_ok:
                return image

    def __get_light_data(self):
        '''
        Returns light opacity from light sensor
        '''
        while True:
            res, _, light_opacity,_ ,_ = exec_vrep_script(self.client_id, 'light_sensor', 'get_light')
            if res == sim.simx_return_ok:
                return light_opacity[0]

    def get_reading(self, pin: int) -> list:
        '''
        Gets reading of a specific sensor
        Param: pin: the pin of the sensor
        Returns: the reading of the requested sensor
        '''
        if pin == 0:
            time.sleep(0.1) # has to have this 'break' else error occurs
            return self.__get_light_data()
        elif pin == 1:
            time.sleep(0.1)
            return self.__get_line_data('MiddleSensor')
        elif pin == 2:
            time.sleep(0.1)
            return self.__get_line_data('RightSensor')
        elif pin == 3:
            time.sleep(0.1)
            return self.__get_line_data('LeftSensor')


class Motor(control_interfaces.MotorInterface):
    """
    Motor control
    Motor(client_id,motor_joint_name,def_speed)
    """

    def __init__(self, client_id: int, motor_joint_name: str, def_speed: int):
        self.client_id = client_id
        self.motor_name = motor_joint_name
        self.def_speed = def_speed

    def __change_motor_velocity(self, velocity: float) -> int:
        '''
        Changes a motor's velocity
        Param: motor: the motor of the simulation to change its velocity (example 'left_motor')
                velocity: the velocity to be changed to
        Returns: a return code of the API function
        '''
        while True:
            res, _, _,_ ,_ = exec_vrep_script(self.client_id, self.motor_name, 'change_vel', inFloats=[velocity])
            if res == sim.simx_return_ok:
                return res

    def dir_control(self, direction: str) -> None:
        '''
        Change motor direction
        Param: direction: the direction to be headed to
        '''
        if direction == 'forward':
            self.__change_motor_velocity(-self.def_speed)
        elif direction == "reverse":
            self.__change_motor_velocity(self.def_speed)
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
            self.__change_motor_velocity(self.def_speed)

    def stop(self) -> None:
        '''Stops the motor'''
        self.__change_motor_velocity(0)


class Odometer(control_interfaces.OdometerInterface):
    '''
    Class Odometer() -> Odometer control
    Functions:
    count_revolutions() Increases the counter of revolutions
    get_revolutions() Returns the number of revolutions
    get_distance() Returns the traveled distance in cm
    reset() Resets the steps counter
    '''
    def __init__(self, client_id: int, motor_name: str):
        self.sensor_disc = 20   #by default 20 lines sensor disc
        self.steps = 0
        self.wheel_diameter = 6.65  #by default the wheel diameter is 6.6
        self.precision = 2  #by default the distance is rounded in 2 digits
        self.client_id = client_id
        self.motor_name = motor_name

    def count_revolutions(self) -> None:
        ''' Increase total steps by one '''
        while True:
            res, steps, _, _, _ = exec_vrep_script(self.client_id, self.motor_name, 'count_revolutions')
            if res == sim.simx_return_ok:
                self.steps = steps[0]
                break

    def get_steps(self) -> int:
        ''' Returns total number of steps '''
        while True:
            res, steps, _, _, _ = exec_vrep_script(self.client_id, self.motor_name, 'get_steps')
            if res == sim.simx_return_ok:
                self.steps = steps[0]
                return self.steps

    def get_revolutions(self) -> float:
        ''' Return total number of revolutions '''
        self.steps = self.get_steps()
        return self.steps / self.sensor_disc

    def __print_distance(self, distance) -> None:
        ''' Prints distance, used for debugging '''
        if self.motor_name == 'right_motor':
            print(f'Distance: {distance}')

    def get_distance(self) -> float:
        ''' Return the total distance so far (in cm) '''
        self.steps = self.get_steps()
        circumference = self.wheel_diameter * math.pi
        revolutions = self.steps / self.sensor_disc
        distance = revolutions * circumference
        #self.__print_distance(distance) # used only for debugging
        return (round(distance, self.precision))

    def reset(self) -> None:
        ''' Reset the total distance and revolutions '''
        while True:
            res, _, _, _, _ = exec_vrep_script(self.client_id, self.motor_name, 'reset_steps')
            if res == sim.simx_return_ok:
                break
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
        max_dist = 999.9
        while True:
            res, handle, distance,_ ,_ = exec_vrep_script(self.client_id, 'ultrasonic_sensor', 'get_distance')
            if res == sim.simx_return_ok:
                break
        #Detected Handle: handle[0], Distance (in meters): distance[0]
        if distance[0] >= 1:
            return max_dist
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
        '''
        Creates dictionary out of list of x, y, z forces
        Param: force_list: the list of x, y, z forces
        Returns: the dictionary of x, y, z forces
        '''
        return {'x': force_list[0], 'y': force_list[1], 'z': force_list[2]}

    def get_acceleration(self, dimension: str = "all") -> dict:
        '''
        Gets the acceleration for a specific or all dimensions
        Param: dimension: the dimension requested (can be 'all')
        Returns: the acceleration for a specific or all dimensions 
        '''
        while True:
            # 1st response -> function executed correctly, 2ns response -> data was successfully collected
            res_1, res_2, accel_data,_ ,_ = exec_vrep_script(self.client_id, 'Accelerometer', 'get_accel')
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
        '''
        Gets gyroscope for a specific or all dimensions
        Param: dimension: the dimension requested (can be 'all')
        Returns: the gyroscope for a specific or all dimensions 
        '''
        while True:
            res, _, gyro_data,_ ,_ = exec_vrep_script(self.client_id, 'GyroSensor', 'get_gyro')
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
        '''
        Changes the color of a led
        Param: color: the wanted color
        For closing the led, use color == 'closed'
        '''
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

        while True:
            res, _, _, _, _ = exec_vrep_script(self.client_id, 'led_light', 'set_color_led', inFloats=color_rbg)
            if res == sim.simx_return_ok:
                break