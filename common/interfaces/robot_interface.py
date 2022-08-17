"""
Interface for FossBot
"""

from abc import ABC, abstractmethod

class FossBotInterface(ABC):
    """ FossBot Interface """

    @abstractmethod
    def get_distance(self) -> float:
        """ Return Distance in cm  """

    @abstractmethod
    def check_for_obstacle(self) -> bool:
        """ Return True if obstacle detected """

    @abstractmethod
    def just_move(self, direction: str = "forward") -> None:
        """ Move input direction forever """

    @abstractmethod
    def stop(self) -> None:
        """ Stop moving """

    @abstractmethod
    def get_floor_sensor(self, sensor_id: int) -> list:
        """ Returns reading of analogue reader """

    @abstractmethod
    def check_on_line(self, sensor_id: int) -> bool:
        """ Returns True if on line """

    @abstractmethod
    def get_light_sensor(self) -> list:
        """ Returns analogue reader """

    @abstractmethod
    def check_for_dark(self) -> bool:
        """ Returns True if dark is detected """

    @abstractmethod
    def wait(self, time_s: int) -> None:
        """ Sleeps for time_s time """

    @abstractmethod
    def move_forward_distance(self, dist: int) -> None:
        """ Moves forward dist """

    @abstractmethod
    def move_forward_default(self) -> None:
        """ Moves forward default distance """

    @abstractmethod
    def rotate_clockwise(self) -> None:
        """ Rotates clockwise """

    @abstractmethod
    def rotate_counterclockwise(self) -> None:
        """ Rotates counterclockwise """

    @abstractmethod
    def move_forward(self) -> None:
        """ Just moves forward (default speed) """

    @abstractmethod
    def rotate_clockwise_90(self) -> None:
        """ Rotates clockwise 90 degrees """

    @abstractmethod
    def rotate_counterclockwise_90(self) -> None:
        """ Rotates counterclockwise 90 degrees """

    @abstractmethod
    def get_noise_detection(self) -> bool:
        """ Returns True if noise detected """

    @abstractmethod
    def move_reverse_distance(self, dist: int) -> None:
        """ Moves dist in reverse """

    @abstractmethod
    def move_reverse_default(self) -> None:
        """ Moves default distance in reverse """

    @abstractmethod
    def move_reverse(self) -> None:
        """ Just moves in reverse (default speed) """

    @abstractmethod
    def play_sound(self, audio_id: int) -> None:
        """ Plays audio_id sound """

    @abstractmethod
    def rgb_set_color(self, color: str) -> None:
        """ Sets color of led """

    @abstractmethod
    def just_rotate(self, dir_id: int) -> None:
        """ Rotates forever """

    @abstractmethod
    def move_distance(self, dist: int, direction: str = "forward") -> None:
        """ Moves dist forwards or backwards """

    @abstractmethod
    def reset_dir(self) -> None:
        """ Resets direction of motors to default (forwards) """

    @abstractmethod
    def rotate_90(self, dir_id: int) -> None:
        """ Rotates 90 degrees """

    @abstractmethod
    def get_acceleration(self, axis: str) -> dict:
        """ Gets acceleration of an axis """

    @abstractmethod
    def get_gyroscope(self, axis: str) -> dict:
        """ Gets gyroscope of an axis """

    @abstractmethod
    def exit(self) -> None:
        """ Exits """
