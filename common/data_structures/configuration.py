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
class RobotParameters:
    """ Dataclass for all parameters """
    sensor_distance: SensorDistance
    motor_left_speed: MotorLeftSpeed
    motor_right_speed: MotorRightSpeed
    default_step: DefaultStep
