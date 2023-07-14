"""
Implementation for godot robot.
"""
import time
import socketio
from fossbot_lib.common.interfaces import robot_interface
from fossbot_lib.godot_robot import control
from fossbot_lib.godot_robot.godot_handler import GodotHandler

class FossBot(robot_interface.FossBotInterface):
    """ Godot robot """

    def __init__(self, session_id: str, **kwargs) -> None:
        """
        Initializes a Godot Fossbot Object with the provided session ID and optional parameters.
        Param:
            session_id (str): The session ID of the Godot simulator in the browser.
            kwargs (optional):
             - server_address (str): The address of the server. Defaults to 'http://localhost:8000'.
             - vel_left (float): The velocity of the left motor. Defaults to 100.
             - vel_right (float): The velocity of the right motor. Defaults to 100.
             - default_dist (float): The default distance. Defaults to 15.
             - degree (float): The degree value. Defaults to 90.
             - light_value (float): The light value. Defaults to 700.
             - middle_sensor_val (float): The value of the middle sensor for black line detection. Defaults to 50.
             - right_sensor_val (float): The value of the right sensor for black line detection. Defaults to 50.
             - left_sensor_val (float): The value of the left sensor for black line detection. Defaults to 50.
             - sensor_distance (float): The max distance for detecting objects. Defaults to 15.
             - motor_left_name (str): The name of the left motor. Defaults to "motor_left".
             - motor_right_name (str): The name of the right motor. Defaults to "motor_right".
        """
        self.response = None    # leave it as None

        self.session_id = session_id
        self.sio = socketio.Client()

        @self.sio.event
        def connect():
            self.sio.emit('pythonConnect', self.session_id)

        self.godotHandler = GodotHandler(self.sio)

        server_address = self.__check_kwargs(kwargs, "server_address", 'http://localhost:8000')

        self.sio.connect(server_address)
        print(f"Connected to socketio server on {server_address}")

        self.vel_left = self.__check_kwargs(kwargs, "vel_left", 100)
        self.vel_right = self.__check_kwargs(kwargs, "vel_right", 100)
        self.default_dist = self.__check_kwargs(kwargs, "default_dist", 15)
        self.degree = self.__check_kwargs(kwargs, "degree", 90)
        self.light_value = self.__check_kwargs(kwargs, "light_value", 700)
        self.middle_sensor_val = self.__check_kwargs(kwargs, "middle_sensor_val", 50)
        self.right_sensor_val = self.__check_kwargs(kwargs, "right_sensor_val", 50)
        self.left_sensor_val = self.__check_kwargs(kwargs, "left_sensor_val", 50)
        self.sensor_distance = self.__check_kwargs(kwargs, "sensor_distance", 15)
        self.motor_left_name = self.__check_kwargs(kwargs, "motor_left_name", "motor_left")
        self.motor_right_name = self.__check_kwargs(kwargs, "motor_right_name", "motor_right")

        self.godotHandler.post_godot(param={"func":"set_motor_names", "right_motor_name": self.motor_right_name, "left_motor_name": self.motor_left_name})

        self.accelerometer = control.Accelerometer(self.godotHandler)
        self.rgb_led = control.LedRGB(self.godotHandler)
        self.analogue_reader = control.AnalogueReadings(self.godotHandler)
        self.noise = control.Noise(self.godotHandler)
        self.ultrasonic = control.UltrasonicSensor(self.godotHandler)
        self.timer = control.Timer(self.godotHandler)
        self.motor_right = control.Motor(self.godotHandler, self.motor_right_name, self.vel_left)
        self.motor_left = control.Motor(self.godotHandler, self.motor_left_name, self.vel_right)
        self.odometer_right = control.Odometer(self.godotHandler, self.motor_right_name)
        self.odometer_left = control.Odometer(self.godotHandler, self.motor_left_name)


    def __check_kwargs(self, kwargs, arg_name: str, def_value):
        if arg_name not in kwargs:
            return def_value
        return kwargs[arg_name]

    # movement
    def just_move(self, direction: str = "forward") -> None:
        """
        Move forward/backwards.
        Param: direction: the direction to be headed to.
        """
        if direction not in ["forward", "reverse"]:
            print('Uknown Direction!')
            raise RuntimeError
        param = {
            "func": "just_move",
            "vel_left": self.vel_left,
            "vel_right": self.vel_right,
            "direction": direction
        }
        self.godotHandler.post_godot(param)

    def move_distance(self, dist: float, direction: str = "forward") -> None:
        '''
        Moves to input direction (default == forward) a specified - input distance (cm).
        Param: dist: the distance to be moved (in cm).
               direction: the direction to be moved towards.
        '''
        if dist < 0:
            print("Negative Distance not allowed.")
            raise RuntimeError
        if direction not in ["forward", "reverse"]:
            print('Uknown Direction!')
            raise RuntimeError
        if dist == 0:
            return
        param = {
            "func": "move_distance",
            "vel_left": self.vel_left,
            "vel_right": self.vel_right,
            "tar_dist": dist,
            "direction": direction
        }
        self.godotHandler.post_godot(param)
        dis_run_r = self.odometer_right.get_distance()
        while dis_run_r < dist:
            dis_run_r = self.odometer_right.get_distance()
        self.stop()

    def reset_dir(self) -> None:
        '''
        Resets all motors direction to default (forward).
        '''
        param = {
            "func": "reset_dir"
        }
        self.godotHandler.post_godot(param)

    def stop(self) -> None:
        """ Stop moving. """
        param = {
            "func": "stop"
        }
        self.godotHandler.post_godot(param)

    def wait(self, time_s: int) -> None:
        '''
        Waits (sleeps) for an amount of time.
        Param: time_s: the time (seconds) of sleep.
        '''
        param = {
            "func": "wait",
            "wait_time": time_s
        }
        self.godotHandler.post_godot(param)
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
        self.move_distance(self.default_dist)

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
        self.move_distance(self.default_dist, direction="reverse")

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
        if dir_id not in [0, 1]:
            print('Uknown Direction!')
            raise RuntimeError
        param = {
            "func": "just_rotate",
            "dir_id": dir_id,
            "vel_right": self.vel_right,
            "vel_left": self.vel_left
        }
        self.godotHandler.post_godot(param)

    def __get_degrees(self) -> float:
        '''Returns degrees of fossbot.'''
        return self.godotHandler.get_godot({"func":"deg_rotated"})


    def rotate_90(self, dir_id: int) -> None:
        '''
        Rotates fossbot 90 degrees towards the specified dir_id.
        Param: dir_id: the direction id to rotate 90 degrees:
                - counterclockwise: dir_id == 0
                - clockwise: dir_id == 1
        '''
        if dir_id not in [0, 1]:
            print('Uknown Direction!')
            raise RuntimeError
        param = {
            "func": "rotate_90",
            "degree": self.degree,
            "dir_id": dir_id,
            "vel_right": self.vel_right,
            "vel_left": self.vel_left
        }
        self.godotHandler.post_godot(param)
        d = self.__get_degrees()
        while d < self.degree:
            d = self.__get_degrees()
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
        i = self.ultrasonic.get_distance()
        if i <= self.sensor_distance:
            return True
        return False

    # sound
    def play_sound(self, audio_path: str) -> None:
        '''
        Plays mp3 file specified by input audio_path.
        Param: audio_path: the path to the wanted mp3 file in godot.
        '''
        param = {
            "func": "play_sound",
            "sound_path": audio_path
        }
        self.godotHandler.post_godot(param)

    # floor sensors
    def get_floor_sensor(self, sensor_id: int) -> float:
        '''
        Gets reading of a floor - line sensor specified by sensor_id.
        Param: sensor_id: the id of the wanted floor - line sensor.
        Returns: the reading of input floor - line sensor.
        '''
        if sensor_id not in [1, 2, 3]:
            print(f'Sensor id {sensor_id} is out of bounds.')
            return 0.0
        return self.analogue_reader.get_reading(sensor_id)

    def check_on_line(self, sensor_id: int) -> bool:
        '''
        Checks if line sensor (specified by sensor_id) is on black line.
        Param: sensor_id: the id of the wanted floor - line sensor.
        Returns: True if sensor is on line, else False.
        '''
        if sensor_id not in [1, 2, 3]:
            print(f'Sensor id {sensor_id} is out of bounds.')
            return False

        read = self.analogue_reader.get_reading(sensor_id)
        #print(read)
        if sensor_id == 1:
            if read <= self.middle_sensor_val / 100:
                return True
        elif sensor_id == 2:
            if read <= self.right_sensor_val / 100:
                return True
        elif sensor_id == 3:
            if read <= self.left_sensor_val / 100:
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
        value = self.get_light_sensor()
        print(value)
        return bool(value < self.light_value)

    # noise detection
    def get_noise_detection(self) -> bool:
        """ Returns True only if noise is detected """
        state = self.noise.detect_noise()
        print(state)
        return state

    # exit
    def exit(self) -> None:
        ''' Exits. '''
        self.stop()
        self.rgb_set_color('closed')
        self.sio.disconnect()

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
