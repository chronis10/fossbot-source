"""
Interface for FossBot.
"""

from abc import ABC, abstractmethod

class FossBotInterface(ABC):
    """ FossBot Interface """

    # movement
    @abstractmethod
    def just_move(self, direction: str = "forward") -> None:
        """
        Move forward/backwards forever.
        Param: direction: the direction to be headed to.
        """

    @abstractmethod
    def move_distance(self, dist: int, direction: str = "forward") -> None:
        '''
        Moves to input direction (default == forward) a specified - input distance (cm).
        Param: dist: the distance to be moved (in cm).
               direction: the direction to be moved towards.
        '''

    @abstractmethod
    def reset_dir(self) -> None:
        '''
        Resets all motors direction to default (forward).
        '''

    @abstractmethod
    def stop(self) -> None:
        """ Stop moving. """

    @abstractmethod
    def wait(self, time_s: int) -> None:
        '''
        Waits (sleeps) for an amount of time.
        Param: time_s: the time (seconds) of sleep.
        '''

    # moving forward
    @abstractmethod
    def move_forward_distance(self, dist: int) -> None:
        '''
        Moves robot forward input distance.
        Param: dist: the distance (cm) to be moved by robot.
        '''

    @abstractmethod
    def move_forward_default(self) -> None:
        '''
        Moves robot forward default distance.
        '''

    @abstractmethod
    def move_forward(self) -> None:
        '''
        Moves robot forwards (forever).
        '''

    # moving reverse
    @abstractmethod
    def move_reverse_distance(self, dist: int) -> None:
        '''
        Moves robot input distance in reverse.
        Param: dist: the distance (cm) to be moved by robot.
        '''

    @abstractmethod
    def move_reverse_default(self) -> None:
        '''
        Moves robot default distance in reverse.
        '''

    @abstractmethod
    def move_reverse(self) -> None:
        '''
        Moves robot in reverse (forever).
        '''

    # rotation
    @abstractmethod
    def just_rotate(self, dir_id: int) -> None:
        '''
        Rotates fossbot towards the specified dir_id (forever).
        Param: dir_id: the direction id to rotate to:
               - clockwise: dir_id == 0
               - counterclockwise: dir_id == 1
        '''

    @abstractmethod
    def rotate_90(self, dir_id: int) -> None:
        '''
        Rotates fossbot 90 degrees towards the specified dir_id.
        Param: dir_id: the direction id to rotate 90 degrees:
               - clockwise: dir_id == 0
               - counterclockwise: dir_id == 1
        '''

    @abstractmethod
    def rotate_clockwise(self) -> None:
        '''
        Rotates robot clockwise (forever).
        '''

    @abstractmethod
    def rotate_counterclockwise(self) -> None:
        '''
        Rotates robot counterclockwise (forever).
        '''

    @abstractmethod
    def rotate_clockwise_90(self) -> None:
        '''
        Rotates robot 90 degrees clockwise.
        '''

    @abstractmethod
    def rotate_counterclockwise_90(self) -> None:
        '''
        Rotates robot 90 degrees counterclockwise.
        '''

    # ultrasonic sensor
    @abstractmethod
    def get_distance(self) -> float:
        '''Returns distance of nearest obstacle in cm.'''

    @abstractmethod
    def check_for_obstacle(self) -> bool:
        '''Returns True only if an obstacle is detected.'''

    # sound
    @abstractmethod
    def play_sound(self, audio_id: int) -> None:
        '''
        Plays mp3 file specified by input audio_id.
        '''

    # floor sensors
    @abstractmethod
    def get_floor_sensor(self, sensor_id: int) -> float:
        '''
        Gets reading of a floor - line sensor specified by sensor_id.
        Param: sensor_id: the id of the wanted floor - line sensor.
        Returns: the reading of input floor - line sensor.
        '''

    @abstractmethod
    def check_on_line(self, sensor_id: int) -> bool:
        '''
        Checks if line sensor (specified by sensor_id) is on black line.
        Param: sensor_id: the id of the wanted floor - line sensor.
        Returns: True if sensor is on line, else False.
        '''

    # accelerometer
    @abstractmethod
    def get_acceleration(self, axis: str) -> float:
        '''
        Gets acceleration of specified axis.
        Param: axis: the axis to get the acceleration from.
        Returns: the acceleration of specified axis.
        '''

    @abstractmethod
    def get_gyroscope(self, axis: str) -> float:
        '''
        Gets gyroscope of specified axis.
        Param: axis: the axis to get the gyroscope from.
        Returns: the gyroscope of specified axis.
        '''

    # rgb
    @abstractmethod
    def rgb_set_color(self, color: str) -> None:
        '''
        Sets a led to input color.
        Param: color: the wanted color.
        '''

    # light sensor
    @abstractmethod
    def get_light_sensor(self) -> float:
        '''
        Returns the reading of the light sensor.
        '''

    @abstractmethod
    def check_for_dark(self) -> bool:
        '''
        Returns True only if light sensor detects dark.
        '''

    # noise detection
    @abstractmethod
    def get_noise_detection(self) -> bool:
        """ Returns True only if noise is detected. """

    # exit
    @abstractmethod
    def exit(self) -> None:
        """ Exits. """

    # implemented only in simulation
    @abstractmethod
    def check_collision(self) -> bool:
        '''
        Returns True if robot collides with other (collidable) object.
        '''

    @abstractmethod
    def teleport(self, pos_x: float, pos_y: float, height: float = 0.19, in_bounds: bool = True) -> None:
        '''
        Teleports fossbot to input location.
        Param: pos_x: the x position to teleport to.
               pos_y: the y position to teleport to.
               hegiht: the height to teleport to (default == 0.19).
               in_bounds: if True, fossbot does not fall off the floor bounds.
        '''

    @abstractmethod
    def teleport_random(self, in_bounds: bool = True) -> None:
        '''
        Teleports fossbot to random location.
        Param: in_bounds: if True, fossbot does not fall off the floor bounds.
        '''

    @abstractmethod
    def check_in_bounds(self) -> bool:
        '''Returns True only if fossbot is on the floor.'''

    @abstractmethod
    def reset_orientation(self) -> None:
        '''Resets fossbot orientation (if it has flipped etc).'''
