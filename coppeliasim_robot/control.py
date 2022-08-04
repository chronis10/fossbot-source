"""
Implementation of simulated control
"""

import math
from random import randrange
import time
from common.interfaces import control_interfaces


class Motor(control_interfaces.MotorInterface):
    """ Motor control """

    def __init__(self, motor_joint_name: str):
        raise NotImplementedError

    def control_speed(self, speed: int) -> None:
        """ Set speed immediately 0-100% range """
        raise NotImplementedError

    def set_speed(self, speed: int) -> None:
        """ Set default speed 0-100% range """
        raise NotImplementedError

    def dir_control(self, direction: str) -> None:
        """ Change motor direction """
        raise NotImplementedError

    def move(self, direction: str = "forward") -> None:
        """ Start motor to move with default speed """
        raise NotImplementedError

    def stop(self) -> None:
        """ Stops the motor"""
        raise NotImplementedError


class Odometer(control_interfaces.MotorInterface):
    """ Odometer control"""

    def __init__(self):
        raise NotImplementedError

    def get_state(self) -> None:
        """ Return 0 or 1 """
        raise NotImplementedError

    def count_revolutions(self) -> None:
        """ Increase total revolutions by one  """
        raise NotImplementedError

    def get_steps(self) -> int:
        """ Return total number of steps """
        raise NotImplementedError

    def get_revolutions(self) -> float:
        """ Return total number of revolutions """
        raise NotImplementedError

    def get_distance(self) -> float:
        """ Return the total distance so far """
        raise NotImplementedError

    def reset(self) -> None:
        """ Reset the total distance and revolutions """
        raise NotImplementedError


class UltrasonicSensor(control_interfaces.UltrasonicSensorInterface):
    """ Ultrasonic sensor """

    def __init__(self):
        raise NotImplementedError

    def get_distance(self) -> float:
        """ Return distance on cm """
        raise NotImplementedError
