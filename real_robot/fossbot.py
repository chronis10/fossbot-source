"""
Real robot implementation
"""
import time
import subprocess

from common.data_structures import configuration
from common.interfaces import robot_interface
from real_robot import control


class FossBot(robot_interface.FossBotInterface):
    """ Real robot """

    def __init__(self, parameters: configuration.RobotParameters):
        control.start_lib()
        self.motor_right = control.Motor(speed_pin=23, terma_pin=27, termb_pin=22,
                                         dc_value=parameters.motor_right_speed.value)
        self.motor_left = control.Motor(speed_pin=25, terma_pin=17, termb_pin=24,
                                        dc_value=parameters.motor_right_speed.value)
        self.ultrasonic = control.UltrasonicSensor(echo_pin=5, trig_pin=6)
        self.odometer_right = control.Odometer(pin=21)
        self.odometer_left = control.Odometer(pin=20)
        self.rgb_led = control.LedRGB()
        self.analogue_reader = control.AnalogueReadings()
        self.accelerometer = control.Accelerometer()
        self.noise = control.GenInput(pin=4)
        self.parameters = parameters

    #ultrasonic sensor
    def get_distance(self) -> float:
        return self.ultrasonic.get_distance()

    def check_for_obstacle(self) -> bool:
        return bool(self.ultrasonic.get_distance() <= self.parameters.sensor_distance.value)

    #floor sensors
    def get_floor_sensor(self, sensor_id: int) -> list:
        return self.analogue_reader.get_reading(sensor_id)


    def check_on_line(self, sensor_id: int) -> bool:
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

    #light sensor
    def get_light_sensor(self) -> list:
        return self.analogue_reader

    def check_for_dark(self) -> bool:
        value = self.analogue_reader.get_reading(0)
        print(value)
        return bool(value >= self.parameters.light_sensor.value)

    def wait(self, time_s: int) -> None:
        time.sleep(time_s)

    #moving forward

    def move_forward_distance(self, dist: int) -> None:
        self.move_distance(dist)

    def move_forward_default(self) -> None:
        self.move_distance(self.parameters.default_step.value)

    def rotate_clockwise(self) -> None:
        self.just_rotate(1)

    def rotate_counterclockwise(self) -> None:
        self.just_rotate(0)

    def move_forward(self) -> None:
        self.just_move()

    def rotate_clockwise_90(self) -> None:
        self.rotate_90(1)

    def rotate_counterclockwise_90(self) -> None:
        self.rotate_90(0)


    def get_noise_detection(self) -> bool:
        state = self.noise.get_state()
        print(state)
        return bool(state) #not bool(state)


    #moving reverse

    def move_reverse_distance(self, dist: int) -> None:
        self.move_distance(dist, direction="reverse")

    def move_reverse_default(self) -> None:
        self.move_distance(self.parameters.default_step.value, direction="reverse")

    def move_reverse(self) -> None:
        self.just_move(direction="reverse")

    #sound
    def play_sound(self, audio_id: int) -> None:
        audio_id = int(audio_id)
        if audio_id == 1:
            subprocess.run(["mpg123", "../robot_lib/soundfx/geia.mp3"], check=True)
        elif audio_id == 2:
            subprocess.run(["mpg123", "../robot_lib/soundfx/mpravo.mp3"], check=True)
        elif audio_id == 3:
            subprocess.run(["mpg123", "../robot_lib/soundfx/empodio.mp3"], check=True)
        elif audio_id == 4:
            subprocess.run(["mpg123", "../robot_lib/soundfx/kalhmera.mp3"], check=True)
        elif audio_id == 5:
            subprocess.run(["mpg123", "../robot_lib/soundfx/euxaristw.mp3"], check=True)
        elif audio_id == 6:
            subprocess.run(["mpg123", "../robot_lib/soundfx/r2d2.mp3"], check=True)
        elif audio_id == 7:
            subprocess.run(["mpg123", "../robot_lib/soundfx/machine_gun.mp3"], check=True)

    #rgb
    def rgb_set_color(self, color: str) -> None:
        self.rgb_led.set_on(color)

    def just_move(self, direction: str = "forward") -> None:
        self.odometer_right.reset()
        self.motor_left.move(direction=direction)
        self.motor_right.move(direction=direction)

    def just_rotate(self, dir_id: int) -> None:
        self.odometer_right.reset()
        left_dir = "reverse" if dir_id == 1 else "forward"
        right_dir = "reverse" if dir_id == 0 else "forward"
        self.motor_left.move(direction=left_dir)
        self.motor_right.move(direction=right_dir)

    def move_distance(self, dist: int, direction: str = "forward") -> None:
        self.just_move(direction=direction)
        while self.odometer_right.get_distance() < dist:
            time.sleep(0.01)
        self.stop()

    def stop(self) -> None:
        self.motor_left.stop()
        self.motor_right.stop()
        self.reset_dir()

    def reset_dir(self) -> None:
        self.motor_left.dir_control("forward")
        self.motor_right.dir_control("forward")

    def rotate_90(self, dir_id: int) -> None:
        self.just_rotate(dir_id)
        rotations = self.parameters.rotate_90.value
        while self.odometer_right.get_steps() <= rotations:
            time.sleep(0.01)
        self.stop()

    def get_acceleration(self, axis: str) -> dict:
        value = self.accelerometer.get_acceleration(dimension=axis)
        print(value)
        return value

    def get_gyroscope(self, axis: str) -> dict:
        value = self.accelerometer.get_gyro(dimension=axis)
        print(value)
        return value

    def __del__(self):
        control.clean()

    def exit(self) -> None:
        control.clean()
