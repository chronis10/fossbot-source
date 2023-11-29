"""
Real robot implementation
"""
import time
import os
import subprocess
from fossbot_lib.common.data_structures import configuration
from fossbot_lib.common.interfaces import robot_interface
from fossbot_lib.real_robot import control


class FossBot(robot_interface.FossBotInterface):
    """ Real robot """

    def __init__(self, parameters: configuration.RobotParameters) -> None:
        control.start_lib()
        self.motor_right = control.Motor(speed_pin=23, terma_pin=27, termb_pin=22,
                                         dc_value=parameters.motor_right_speed.value)
        self.motor_left = control.Motor(speed_pin=25, terma_pin=17, termb_pin=24,
                                        dc_value=parameters.motor_left_speed.value)
        self.ultrasonic = control.UltrasonicSensor(echo_pin=5, trig_pin=6)
        self.odometer_right = control.Odometer(pin=21)
        self.odometer_left = control.Odometer(pin=20)
        self.rgb_led = control.LedRGB()
        self.analogue_reader = control.AnalogueReadings()
        self.accelerometer = control.Accelerometer()
        self.noise = control.Noise(pin=4)
        self.timer = control.Timer()
        self.parameters = parameters

    # movement
    def just_move(self, direction: str = "forward") -> None:
        """
        Move forward/backwards.
        Param: direction: the direction to be headed to.
        """
        self.odometer_right.reset()
        self.motor_left.move(direction=direction)
        self.motor_right.move(direction=direction)

    def move_distance(self, dist: float, direction: str = "forward") -> None:
        '''
        Moves to input direction (default == forward) a specified - input distance (cm).
        Param: dist: the distance to be moved (in cm).
               direction: the direction to be moved towards.
        '''
        self.just_move(direction=direction)
        while self.odometer_right.get_distance() < dist:
            time.sleep(0.01)
        self.stop()

    def reset_dir(self) -> None:
        '''
        Resets all motors direction to default (forward).
        '''
        self.motor_left.dir_control("forward")
        self.motor_right.dir_control("forward")

    def stop(self) -> None:
        """ Stop moving. """
        self.motor_left.stop()
        self.motor_right.stop()
        self.reset_dir()

    def wait(self, time_s: int) -> None:
        '''
        Waits (sleeps) for an amount of time.
        Param: time_s: the time (seconds) of sleep.
        '''
        time.sleep(time_s)

    # moving forward
    def move_forward_distance(self, dist: float) -> None:
        '''
        Moves robot forward input distance.
        Param: dist: the distance (cm) to be moved by robot.
        '''
        self.move_distance(dist)

    def move_forward_default(self) -> None:
        '''
        Moves robot forward default distance.
        '''
        self.move_distance(self.parameters.default_step.value)

    def move_forward(self) -> None:
        '''
        Moves robot forwards.
        '''
        self.just_move()

    # moving reverse
    def move_reverse_distance(self, dist: float) -> None:
        '''
        Moves robot input distance in reverse.
        Param: dist: the distance (cm) to be moved by robot.
        '''
        self.move_distance(dist, direction="reverse")

    def move_reverse_default(self) -> None:
        '''
        Moves robot default distance in reverse.
        '''
        self.move_distance(self.parameters.default_step.value, direction="reverse")

    def move_reverse(self) -> None:
        '''
        Moves robot in reverse.
        '''
        self.just_move(direction="reverse")

    # rotation
    def just_rotate(self, dir_id: int) -> None:
        '''
        Rotates fossbot towards the specified dir_id.
        Param: dir_id: the direction id to rotate to:
                - counterclockwise: dir_id == 0
                - clockwise: dir_id == 1
        '''
        self.odometer_right.reset()
        left_dir = "reverse" if dir_id == 0 else "forward"
        right_dir = "reverse" if dir_id == 1 else "forward"
        self.motor_left.move(direction=left_dir)
        self.motor_right.move(direction=right_dir)

    def rotate_90(self, dir_id: int) -> None:
        '''
        Rotates fossbot 90 degrees towards the specified dir_id.
        Param: dir_id: the direction id to rotate 90 degrees:
                - counterclockwise: dir_id == 0
                - clockwise: dir_id == 1
        '''
        self.just_rotate(dir_id)
        rotations = self.parameters.rotate_90.value
        while self.odometer_right.get_steps() <= rotations:
            time.sleep(0.01)
        self.stop()

    def rotate_clockwise(self) -> None:
        '''
        Rotates robot clockwise.
        '''
        self.just_rotate(1)

    def rotate_counterclockwise(self) -> None:
        '''
        Rotates robot counterclockwise.
        '''
        self.just_rotate(0)

    def rotate_clockwise_90(self) -> None:
        '''
        Rotates robot 90 degrees clockwise.
        '''
        self.rotate_90(1)

    def rotate_counterclockwise_90(self) -> None:
        '''
        Rotates robot 90 degrees counterclockwise.
        '''
        self.rotate_90(0)

    # ultrasonic sensor
    def get_distance(self) -> float:
        '''Returns distance of nearest obstacle in cm.'''
        return self.ultrasonic.get_distance()

    def check_for_obstacle(self) -> bool:
        '''Returns True only if an obstacle is detected.'''
        return bool(self.ultrasonic.get_distance() <= self.parameters.sensor_distance.value)

    # sound
    def play_sound(self, audio_path: str) -> None:
        '''
        Plays mp3 file specified by input audio_path.
        Param: audio_path: the path to the wanted mp3 file.
        '''
        audio_path = os.path.normpath(audio_path)
        subprocess.run(["mpg123",audio_path])

    # floor sensors
    def get_floor_sensor(self, sensor_id: int) -> float:
        '''
        Gets reading of a floor - line sensor specified by sensor_id.
        Param: sensor_id: the id of the wanted floor - line sensor.
        Returns: the reading of input floor - line sensor.
        '''
        return self.analogue_reader.get_reading(sensor_id)

    def check_on_line(self, sensor_id: int) -> bool:
        '''
        Checks if line sensor (specified by sensor_id) is on black line.
        Param: sensor_id: the id of the wanted floor - line sensor.
        Returns: True if sensor is on line, else False.
        '''
        sensor_left = self.parameters.line_sensor_left.value
        sensor_center = self.parameters.line_sensor_center.value
        sensor_right = self.parameters.line_sensor_right.value
        if sensor_id == 3:
            if self.analogue_reader.get_reading(sensor_id) >= sensor_left:
                return True
        elif sensor_id == 1:
            if self.analogue_reader.get_reading(sensor_id) >= sensor_center:
                return True
        elif sensor_id == 2:
            if self.analogue_reader.get_reading(sensor_id) >= sensor_right:
                return True
        return False

    # accelerometer
    def get_acceleration(self, axis: str) -> float:
        '''
        Gets acceleration of specified axis.
        Param: axis: the axis to get the acceleration from.
        Returns: the acceleration of specified axis.
        '''
        value = self.accelerometer.get_acceleration(dimension=axis)
        print(value)
        return value

    def get_gyroscope(self, axis: str) -> float:
        '''
        Gets gyroscope of specified axis.
        Param: axis: the axis to get the gyroscope from.
        Returns: the gyroscope of specified axis.
        '''
        value = self.accelerometer.get_gyro(dimension=axis)
        print(value)
        return value

    # rgb
    def rgb_set_color(self, color: str) -> None:
        '''
        Sets a led to input color.
        Param: color: the wanted color.
        '''
        self.rgb_led.set_on(color)

    # light sensor
    def get_light_sensor(self) -> float:
        '''
        Returns the reading of the light sensor.
        '''
        return self.analogue_reader.get_reading(0)

    def check_for_dark(self) -> bool:
        '''
        Returns True only if light sensor detects dark.
        '''
        value = self.analogue_reader.get_reading(0)
        print(value)
        return bool(value >= self.parameters.light_sensor.value)

    # noise detection
    def get_noise_detection(self) -> bool:
        """ Returns True only if noise is detected """
        state = self.noise.detect_noise()
        print(state)
        return state

    # exit
    def exit(self) -> None:
        ''' Exits. '''
        control.clean()

    def __del__(self) -> None:
        control.clean()

    # timer:
    def stop_timer(self) -> None:
        '''Stops the timer.'''
        self.timer.stop_timer()

    def start_timer(self) -> None:
        '''Starts the timer.'''
        self.timer.start_timer()

    def get_elapsed(self) -> int:
        '''Returns the time from start.'''
        value = self.timer.get_elapsed()
        print('elapsed time in sec:', value)
        return value
