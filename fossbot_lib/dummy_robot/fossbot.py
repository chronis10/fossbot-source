"""
Implementation for dummy robot.
"""
import random
from fossbot_lib.common.interfaces import robot_interface


class FossBot(robot_interface.FossBotInterface):
    """ Dummy robot """
    # movement
    def just_move(self, direction: str = "forward") -> None:
        """
        Move forward/backwards.
        Param: direction: the direction to be headed to.
        """
        print('Just moving.')

    def move_distance(self, dist: float, direction: str = "forward") -> None:
        '''
        Moves to input direction (default == forward) a specified - input distance (cm).
        Param: dist: the distance to be moved (in cm).
               direction: the direction to be moved towards.
        '''
        print('Moving distance.')

    def reset_dir(self) -> None:
        '''
        Resets all motors direction to default (forward).
        '''
        print('Direction reset.')

    def stop(self) -> None:
        """ Stop moving. """
        print('Stop.')

    def wait(self, time_s: int) -> None:
        '''
        Waits (sleeps) for an amount of time.
        Param: time_s: the time (seconds) of sleep.
        '''
        print('Sleeping...')

    # moving forward
    def move_forward_distance(self, dist: float) -> None:
        '''
        Moves robot forward input distance.
        Param: dist: the distance (cm) to be moved by robot.
        '''
        print('Moving forward distance.')

    def move_forward_default(self) -> None:
        '''
        Moves robot forward default distance.
        '''
        print('Moving forward default distance.')

    def move_forward(self) -> None:
        '''
        Moves robot forwards.
        '''
        print('Moving forward.')

    # moving reverse
    def move_reverse_distance(self, dist: float) -> None:
        '''
        Moves robot input distance in reverse.
        Param: dist: the distance (cm) to be moved by robot.
        '''
        print('Moving distance in reverse.')

    def move_reverse_default(self) -> None:
        '''
        Moves robot default distance in reverse.
        '''
        print('Moving default distance in reverse.')

    def move_reverse(self) -> None:
        '''
        Moves robot in reverse.
        '''
        print('Moving in reverse.')

    # rotation
    def just_rotate(self, dir_id: int) -> None:
        '''
        Rotates fossbot towards the specified dir_id.
        Param: dir_id: the direction id to rotate to:
               - clockwise: dir_id == 0
               - counterclockwise: dir_id == 1
        '''
        print('Just rotating.')

    def rotate_90(self, dir_id: int) -> None:
        '''
        Rotates fossbot 90 degrees towards the specified dir_id.
        Param: dir_id: the direction id to rotate 90 degrees:
               - clockwise: dir_id == 0
               - counterclockwise: dir_id == 1
        '''
        print('Rotating 90 degrees.')

    def rotate_clockwise(self) -> None:
        '''
        Rotates robot clockwise.
        '''
        print('Rotating clockwise.')

    def rotate_counterclockwise(self) -> None:
        '''
        Rotates robot counterclockwise.
        '''
        print('Rotating counterclockwise.')

    def rotate_clockwise_90(self) -> None:
        '''
        Rotates robot 90 degrees clockwise.
        '''
        print('Rotating clockwise 90 degrees.')

    def rotate_counterclockwise_90(self) -> None:
        '''
        Rotates robot 90 degrees counterclockwise.
        '''
        print('Rotating counterclockwise 90 degrees.')

    # ultrasonic sensor
    def get_distance(self) -> float:
        '''Returns distance of nearest obstacle in cm.'''
        return random.random()

    def check_for_obstacle(self) -> bool:
        '''Returns True only if an obstacle is detected.'''
        return bool(random.randint(0, 1))

    # sound
    def play_sound(self, audio_id: int) -> None:
        '''
        Plays mp3 file specified by input audio_id.
        '''
        print('Playing sound.')

    # floor sensors
    def get_floor_sensor(self, sensor_id: int) -> float:
        '''
        Gets reading of a floor - line sensor specified by sensor_id.
        Param: sensor_id: the id of the wanted floor - line sensor.
        Returns: the reading of input floor - line sensor.
        '''
        return random.random()

    def check_on_line(self, sensor_id: int) -> bool:
        '''
        Checks if line sensor (specified by sensor_id) is on black line.
        Param: sensor_id: the id of the wanted floor - line sensor.
        Returns: True if sensor is on line, else False.
        '''
        return bool(random.randint(0, 1))

    # accelerometer
    def get_acceleration(self, axis: str) -> float:
        '''
        Gets acceleration of specified axis.
        Param: axis: the axis to get the acceleration from.
        Returns: the acceleration of specified axis.
        '''
        return random.random()

    def get_gyroscope(self, axis: str) -> float:
        '''
        Gets gyroscope of specified axis.
        Param: axis: the axis to get the gyroscope from.
        Returns: the gyroscope of specified axis.
        '''
        return random.random()

    # rgb
    def rgb_set_color(self, color: str) -> None:
        '''
        Sets a led to input color.
        Param: color: the wanted color.
        '''
        print('Led color set.')

    # light sensor
    def get_light_sensor(self) -> float:
        '''
        Returns the reading of the light sensor.
        '''
        return random.random()

    def check_for_dark(self) -> bool:
        '''
        Returns True only if light sensor detects dark.
        '''
        return bool(random.randint(0, 1))

    # noise detection
    def get_noise_detection(self) -> bool:
        """ Returns True only if noise is detected """
        return bool(random.randint(0, 1))

    # exit
    def exit(self) -> None:
        ''' Exits. '''
        print('Exit.')
