"""
Interfaces for control parts
"""

from abc import ABC, abstractmethod


class MotorInterface(ABC):
    """ Interface for motor """

    @abstractmethod
    def control_speed(self, speed: int) -> None:
        """ Set speed immediately 0-100% range """

    @abstractmethod
    def set_speed(self, speed: int) -> None:
        """ Set default speed 0-100% range """

    @abstractmethod
    def dir_control(self, direction: str) -> None:
        """ Change motor direction """

    @abstractmethod
    def move(self, direction: str = "forward") -> None:
        """ Start motor to move with default speed """

    @abstractmethod
    def stop(self) -> None:
        """ Stops the motor"""


class OdometerInterface(ABC):
    """ Interface for odometer """

    @abstractmethod
    def get_state(self) -> int:
        """ Return 0 or 1 """

    @abstractmethod
    def count_revolutions(self) -> None:
        """ Increase total revolutions by one  """

    @abstractmethod
    def get_steps(self) -> int:
        """ Return total number of steps """

    @abstractmethod
    def get_revolutions(self) -> float:
        """ Return total number of revolutions """

    @abstractmethod
    def get_distance(self) -> float:
        """ Return the total distance so far """

    @abstractmethod
    def reset(self) -> None:
        """ Reset the total distance and revolutions """


class UltrasonicSensorInterface(ABC):
    """ Interface for ultrasonic sensor """

    def get_distance(self) -> float:
        """ Return distance on cm """
