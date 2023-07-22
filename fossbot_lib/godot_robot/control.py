"""
Implementation for control (godot).
"""
from fossbot_lib.godot_robot.godot_handler import GodotHandler
from fossbot_lib.common.interfaces import control_interfaces

class Timer(control_interfaces.TimerInterface):
    '''
    Class timer()
    Functions:
    stop_timer() Stops a timer.
    start_timer() Starts a timer.
    elapsed() Prints elapsed time from start.
    get_elapsed() Returns the elapsed time between start time and that moment in sec.
    '''
    def __init__(self, godotHandler: GodotHandler):
        self.godotHandler = godotHandler

    def stop_timer(self) -> None:
        '''Stops timer.'''
        param = {
            "func": "stop_timer"
        }
        self.godotHandler.post_godot(param)

    def start_timer(self) -> None:
        '''Starts timer.'''
        param = {
            "func": "start_timer"
        }
        self.godotHandler.post_godot(param)

    def elapsed(self) -> None:
        '''Prints elapsed time from start.'''
        print(self.get_elapsed())

    def get_elapsed(self) -> int:
        '''Returns the elapsed time in seconds.'''
        param = {
            "func": "get_elapsed"
        }
        res = self.godotHandler.get_godot(param)
        print(res)
        return int(res)

class Motor(control_interfaces.MotorInterface):
    """
    Motor() -> Motor control.
    Functions:
    dir_control(direction) Change motor direction to input direction.
    move(direction) Start moving motor with default speed towards input direction.
    set_speed(speed) Set speed immediately 0-100 range.
    stop() Stops the motor.
    """

    def __init__(self, godotHandler: GodotHandler, motor_joint_name: str, def_speed: int) -> None:
        self.motor_name = motor_joint_name
        self.godotHandler = godotHandler
        self.def_speed = def_speed
        self.direction = 'forward'

    def __change_motor_velocity(self, velocity: float) -> int:
        '''
        Changes a motor's velocity.
        Param: motor: the motor of the simulation to change its velocity (example 'left_motor').
               velocity: the velocity to be changed to.
        '''
        param = {
            "func": "move_motor",
            "motor_name": self.motor_name,
            "motor_vel": self.def_speed,
            "direction": self.direction
        }
        self.godotHandler.post_godot(param)

    def set_speed(self, speed: int) -> None:
        '''
        Set speed immediately 0-100 range.
        Param: speed: the range 0 - 100 that speed will be changed to.
        '''
        if speed < 0 or speed > 100:
            print("The motor speed is a percentage of total motor power. Accepted values 0-100.")
        else:
            self.def_speed = speed
            self.move(self.direction)

    def dir_control(self, direction: str) -> None:
        '''
        Change motor direction to input direction.
        Param: direction: the direction to be headed to.
        '''
        if direction not in ['forward', 'reverse']:
            print("Motor accepts only forward and reverse values")
        else:
            self.direction = direction

    def move(self, direction: str = "forward") -> None:
        '''
        Start moving motor with default speed towards input direction.
        Param: direction: the direction to be headed to.
        '''
        if direction in ['forward', "reverse"]:
            self.dir_control(direction)
            self.__change_motor_velocity(self.def_speed)
            return
        print("Motor accepts only forward and reverse values")

    def stop(self) -> None:
        '''Stops the motor.'''
        self.__change_motor_velocity(0)


class Odometer(control_interfaces.OdometerInterface):
    '''
    Class Odometer() -> Odometer control.
    Functions:
    count_revolutions() Increases the counter of revolutions.
    get_revolutions() Returns the number of revolutions.
    get_distance() Returns the traveled distance in cm.
    reset() Resets the steps counter.
    '''

    def __init__(self, godotHandler: GodotHandler, motor_name: str) -> None:
        self.precision = 2  #by default the distance is rounded in 2 digits
        self.godotHandler = godotHandler
        self.motor_name = motor_name

    def count_revolutions(self) -> None:
        '''Increase total steps by one.'''
        self.godotHandler.post_godot({"func": "count_revolutions", "motor_name": self.motor_name})

    def get_steps(self) -> int:
        ''' Returns total number of steps. '''
        return self.godotHandler.get_godot({"func": "get_steps", "motor_name": self.motor_name})

    def get_revolutions(self) -> float:
        ''' Returns total number of revolutions. '''
        return self.godotHandler.get_godot({"func": "get_revolutions", "motor_name": self.motor_name})

    def get_distance(self) -> float:
        ''' Return the total distance traveled so far (in cm). '''
        distance = self.godotHandler.get_godot({"func": "dist_travelled", "motor_name": self.motor_name})
        return round(distance, self.precision)

    def reset(self) -> None:
        ''' Reset the total traveled distance and revolutions. '''
        self.godotHandler.post_godot({"func": "reset_steps"})

class UltrasonicSensor(control_interfaces.UltrasonicSensorInterface):
    '''
    Class UltrasonicSensor() -> Ultrasonic sensor control.
    Functions:
    get_distance() return distance in cm.
    '''

    def __init__(self, godotHandler: GodotHandler):
        self.godotHandler = godotHandler

    def get_distance(self) -> float:
        '''
        Gets the distance to the closest obstacle.
        Returns: the distance to the closest obstacle (in cm).
        '''
        param = {
            "func": "get_distance"
        }
        return self.godotHandler.get_godot(param)


class AnalogueReadings(control_interfaces.AnalogueReadingsInterface):
    '''
    Class AnalogueReadings() -> Handles Analogue Readings.
    Functions:
    get_reading(pin) Gets reading of a specific sensor specified by input pin.
    '''
    def __init__(self, godotHandler: GodotHandler):
        self.godotHandler = godotHandler

    def get_reading(self, pin: int) -> float:
        '''
        Gets reading of a specific sensor specified by input pin.
        Param: pin: the pin of the sensor.
        Returns: the reading of the requested sensor.
        '''
        if pin == 0:    # light sensor
            return self.godotHandler.get_godot({"func":"get_light_sensor"})
        return self.godotHandler.get_godot({"func":"get_floor_sensor","sensor_id":pin})


class LedRGB(control_interfaces.LedRGBInterface):
    '''
    Class LedRGB(socket) -> Led control
    set_on(color): sets led to input color.
    '''
    def __init__(self, godotHandler: GodotHandler):
        self.godotHandler = godotHandler

    def set_on(self, color: str) -> None:
        '''
        Changes the color of a led
        Param: color: the wanted color
        For closing the led, use color == 'closed'
        '''
        if color not in ['red', 'green', 'blue', 'white', 'violet', 'cyan', 'yellow', 'closed']:
            print('Uknown color!')
            raise RuntimeError
        param = {
            "func" : "rgb_set_color",
            "color": color
        }
        self.godotHandler.post_godot(param)


class Accelerometer(control_interfaces.AccelerometerInterface):
    '''
    Class Accelerometer() -> Handles accelerometer and gyroscope.
    Functions:
    get_acceleration(dimension) Returns the acceleration for a specific dimension.
    get_gyro(dimension) Returns the gyroscope for a specific dimension.
    '''
    def __init__(self, godotHandler: GodotHandler):
        self.godotHandler = godotHandler

    def get_acceleration(self, dimension: str) -> float:
        '''
        Gets the acceleration for a specific dimension.
        Param: dimension: the dimension requested.
        Returns: the acceleration for a specific dimension.
        '''
        if dimension not in ('x', 'y', 'z'):
            print("Dimension not recognized!!")
            return 0.0
        param = {
            "func": "get_acceleration",
            "axis": dimension
        }
        return self.godotHandler.get_godot(param)


    def get_gyro(self, dimension: str) -> float:
        '''
        Gets gyroscope for a specific dimension.
        Param: dimension: the dimension requested.
        Returns: the gyroscope for a specific dimension.
        '''
        if dimension not in ('x', 'y', 'z'):
            print("Dimension not recognized!!")
            return 0.0
        param = {
            "func": "get_gyroscope",
            "axis": dimension
        }
        return self.godotHandler.get_godot(param)

class Noise(control_interfaces.NoiseInterface):
    '''
    Class Noise() -> Handles Noise Detection.
    Functions:
    detect_noise(): Returns True only if noise was detected.
    '''
    def __init__(self, godotHandler: GodotHandler):
        self.godotHandler = godotHandler

    def detect_noise(self) -> bool:
        '''
        Returns True only if noise was detected.
        '''
        param = {
            "func": "get_noise_detection"
        }
        return self.godotHandler.get_godot(param)

# Hardware section
class GenInput(control_interfaces.GenInputInterface):
    '''
    Class GenInput().
    Functions:
    get_state(): Returns state 0 or 1.
    '''
    def get_state(self) -> int:
        '''
        Returns state 0 or 1
        '''
        raise NotImplementedError

class GenOutput(control_interfaces.GenOutputInterface):
    '''
    Class GenOutput()
    Functions:
    set_on() set High the output pin
    set_off() set Low the output pin
    '''
    def set_on(self) -> None:
        '''
        Set High the output pin
        '''
        raise NotImplementedError

    def set_off(self) -> None:
        '''
        Set Low the output pin
        '''
        raise NotImplementedError
