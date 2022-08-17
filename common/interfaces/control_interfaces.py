"""
Interfaces for control parts
"""

from abc import ABC, abstractmethod


class AnalogueReadingsInterface(ABC):
    """ Interface for analogue readings """

    @abstractmethod
    def get_readings(self, pin: int) -> list:
        """ Returns the image data """


class LedRGBInterface(ABC):
    """ Interface for led rgb """

    @abstractmethod
    def set_on(self, color: str) -> None:
        """ Changes the color of a led """


class AccelerometerInterface(ABC):
    """ Interface for accelerometer """

    @abstractmethod
    def get_acceleration(self, dimension: str = "all" ) -> dict:
        """ Returns acceleration for input dimension """

    @abstractmethod
    def get_gyro(self, dimension: str = "all" ) -> dict:
        """ Returns gyroscope for input dimension """


class BuzzerInterface(ABC):
    """ Interface for buzzer """

    @abstractmethod
    def beep(self) -> None:
        """ Short tone """

    @abstractmethod
    def timer(self, count: int) -> None:
        """ Timer function with wait and for every 1 sec makes a tone """


class MotorInterface(ABC):
    """ Interface for motor """

    @abstractmethod
    def set_speed(self, speed: int) -> None:
        """ Set speed immediately 0-100% range """

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
