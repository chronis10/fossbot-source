"""
Robot configuration dataclasses
"""

from dataclasses import dataclass


@dataclass
class DefaultStep:
    """ Deafault distance in cm for 1 step """
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
    """ Min distance for obstacle detection  """
    name: str
    value: int
    default: int

@dataclass
class LightSensor:
    """ Light Sensor """
    name:str
    value:int
    default:int

@dataclass
class LineSensorLeft:
    """ Line Sensor left """
    name:str
    value:int
    default:int

@dataclass
class LineSensorCenter:
    """ Line Sensor center """
    name:str
    value:int
    default:int

@dataclass
class LineSensorRight:
    """ Line Sensor right """
    name:str
    value:int
    default:int

@dataclass
class Rotate90:
    """ Rotate 90 degrees """
    name:str
    value:int
    default:int

@dataclass
class RobotParameters:
    """ Dataclass for all parameters """
    sensor_distance: SensorDistance
    motor_left_speed: MotorLeftSpeed
    motor_right_speed: MotorRightSpeed
    default_step: DefaultStep
    light_sensor: LightSensor
    line_sensor_left: LineSensorLeft
    line_sensor_center: LineSensorCenter
    line_sensor_right: LineSensorRight
    rotate_90: Rotate90
