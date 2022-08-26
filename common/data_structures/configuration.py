"""
Robot configuration dataclasses
"""

from dataclasses import dataclass

@dataclass
class DefaultStep:
    """ Default distance in cm for 1 step """
    name: str
    value: int
    default: int

@dataclass
class MotorRightSpeed:
    """ Default right motor speed """
    name: str
    value: int
    default: int

@dataclass
class MotorLeftSpeed:
    """ Default left motor speed """
    name: str
    value: int
    default: int

@dataclass
class SensorDistance:
    """ Min distance for obstacle detection """
    name: str
    value: int
    default: int

@dataclass
class LightSensor:
    """ Light Sensor """
    name: str
    value: int
    default: int

@dataclass
class LineSensorLeft:
    """ Line Sensor left """
    name: str
    value: int
    default: int

@dataclass
class LineSensorCenter:
    """ Line Sensor center """
    name: str
    value: int
    default: int

@dataclass
class LineSensorRight:
    """ Line Sensor right """
    name: str
    value: int
    default: int

@dataclass
class Rotate90:
    """ Rotate 90 degrees """
    name: str
    value: int
    default: int

@dataclass
class SimRobotIds:
    """ Ids for simulation """
    client_id: int
    left_motor_name: str
    right_motor_name: str
    light_sensor_name: str
    sensor_middle_name: str
    sensor_right_name: str
    sensor_left_name: str
    ultrasonic_name: str
    accelerometer_name: str
    gyroscope_name: str
    led_name: str
    light_sensor_id: int = 0
    sensor_middle_id: int = 1
    sensor_right_id: int = 2
    sensor_left_id: int = 3


@dataclass
class RobotParameters:
    """ Dataclass for all real robot parameters """
    sensor_distance: SensorDistance
    motor_left_speed: MotorLeftSpeed
    motor_right_speed: MotorRightSpeed
    default_step: DefaultStep
    light_sensor: LightSensor
    line_sensor_left: LineSensorLeft
    line_sensor_center: LineSensorCenter
    line_sensor_right: LineSensorRight
    rotate_90: Rotate90

@dataclass
class SimRobotParameters(RobotParameters):
    """ Dataclass for all simulated robot parameters """
    simulation: SimRobotIds
    def __init__(self, param_real: RobotParameters, param_sim: SimRobotIds) -> None:
        self.simulation = param_sim
        self.sensor_distance = param_real.sensor_distance
        self.motor_left_speed = param_real.motor_left_speed
        self.motor_right_speed = param_real.motor_right_speed
        self.light_sensor = param_real.light_sensor
        self.line_sensor_left = param_real.line_sensor_left
        self.line_sensor_center = param_real.line_sensor_center
        self.line_sensor_right = param_real.line_sensor_right
        self.rotate_90 = param_real.rotate_90
        self.default_step = param_real.default_step