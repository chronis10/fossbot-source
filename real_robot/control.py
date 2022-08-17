"""
Implementation of electronic parts control
"""

import math
import time
import RPi.GPIO as GPIO
from mpu6050 import mpu6050
import Adafruit_MCP3008
from common.interfaces import control_interfaces

class GenInput():
    '''
    Class gen_input(pin)
    Default pin 4
    Functions:
    get_state() return True/False
    '''
    def __init__(self, pin=4):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.IN)

    def get_state(self):
        '''
        Returns True/False
        '''
        return GPIO.input(self.pin)

class GenOutput():
    '''
    Class gen_output(pin)
    Deafult pin 5
    Functions:
    set_on() set High the output pin
    set_off() set Low the output pin
    '''
    def __init__(self, pin=5):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, False)

    def set_on(self):
        '''
        Set High the output pin
        '''
        GPIO.output(self.pin, True)

    def set_off(self):
        '''
        Set Low the output pin
        '''
        GPIO.output(self.pin, False)



class Motor(control_interfaces.MotorInterface):
    """
    Motor control
    Motor(speed_pin,terma_pin,termb_pinfreq=17,dc=70)
    """

    def __init__(self, speed_pin: int, terma_pin: int, termb_pin: int, dc_value: int = 70):
        GPIO.setup(speed_pin, GPIO.OUT)
        GPIO.setup(terma_pin, GPIO.OUT)
        GPIO.setup(termb_pin, GPIO.OUT)
        self.terma_pin = terma_pin
        self.termb_pin = termb_pin
        self.freq = 17
        self.mot = GPIO.PWM(speed_pin, self.freq)
        self.dc_value = dc_value
        self.dir_control("forward")
        self.mot.start(0)

    def set_speed(self, speed: int) -> None:
        """ Set speed immediately 0-100% range """
        if speed < 0 or speed > 100:
            print(
                "The motor speed is a percentage of total motor power. Accepted values 0-100.")
        else:
            self.dc_value = speed
            self.mot.ChangeDutyCycle(speed)

    def dir_control(self, direction: str) -> None:
        """ Change motor direction """
        if direction == "forward":
            GPIO.output(self.terma_pin, GPIO.HIGH)
            GPIO.output(self.termb_pin, GPIO.LOW)
        elif direction == "reverse":
            GPIO.output(self.terma_pin, GPIO.LOW)
            GPIO.output(self.termb_pin, GPIO.HIGH)
        else:
            print("Motor accepts only forward and reverse values")

    def move(self, direction: str = "forward") -> None:
        """ Start motor to move with default speed """
        self.dir_control(direction)
        self.mot.ChangeDutyCycle(self.dc_value)

    def stop(self) -> None:
        """ Stops the motor """
        self.mot.ChangeDutyCycle(0)


class Odometer(control_interfaces.OdometerInterface):
    """
    Odometer control
    Odometer(pin)
    Event triggered counter
    """

    def __init__(self, pin: int):
        self.pin = pin
        self.prev_pos = self.get_state()
        self.sensor_disc = 20
        self.steps = 0
        self.offset = 0
        self.wheel_diameter = 6.65
        self.precision = 2
        GPIO.setup(pin, GPIO.IN)
        GPIO.add_event_detect(
            self.pin,
            GPIO.RISING,
            callback=self.count_revolutions,
            bouncetime=1)

    def get_state(self) -> int:
        """ Return 0 or 1 current state of odometer """
        return GPIO.input(self.pin)

    def count_revolutions(self) -> None:
        self.steps += 1

    def get_steps(self) -> int:
        return self.steps

    def get_revolutions(self) -> float:
        return self.steps / self.sensor_disc

    def get_distance(self) -> float:
        circumference = self.wheel_diameter * math.pi
        revolutions = self.steps / self.sensor_disc
        distance = revolutions * circumference
        final = round(distance, self.precision)
        return final + self.offset

    def reset(self) -> None:
        """ Reset the total distance and revolutions """
        self.steps = 0


class UltrasonicSensor(control_interfaces.UltrasonicSensorInterface):
    '''
    Ultrasonic Sensor
    UltrasonicSensor(echo_pin,trig_pin)
    '''

    def __init__(self, echo_pin: int = 14, trig_pin: int = 15):
        self.echo_pin = echo_pin
        self.trig_pin = trig_pin
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.output(self.trig_pin, False)

    def get_distance(self) -> float:
        """ Interface for ultrasonic sensor """
        GPIO.output(self.trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, False)
        starttime = time.time()
        stoptime = time.time()
        while GPIO.input(self.echo_pin) == 0:
            starttime = time.time()

        while GPIO.input(self.echo_pin) == 1:
            stoptime = time.time()
        timeelasped = stoptime - starttime
        distance = (timeelasped * 34300) / 2
        return distance


class AnalogueReadings(control_interfaces.AnalogueReadingsInterface):
    '''
    Analogue Readings
    AnalogueReadings(CLK,MISO,MOSI,CS)
    '''

    def __init__(self, CLK: int = 11, MISO: int = 9, MOSI: int = 10, CS: int = 8):
        self.mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

    def get_reading(self, pin: int) -> list:
        value = self.mcp.read_adc(pin)
        print(f'ADC {pin}: {value}')
        return value


class LedRGB(control_interfaces.LedRGBInterface):
    '''
    Led rgb
    LedRGB(pin_r,pin_b,pin_g)
    '''

    def __init__(self, pin_r: int = 16, pin_b: int = 19, pin_g: int = 12):
        self.p_r = GenOutput(pin_r)
        self.p_b = GenOutput(pin_b)
        self.p_g = GenOutput(pin_g)

    def set_on(self, color: str) -> None:
        if color == 'red':
            self.p_r.set_on()
            self.p_b.set_off()
            self.p_g.set_off()
        elif color == 'green':
            self.p_r.set_off()
            self.p_b.set_off()
            self.p_g.set_on()
        elif color == 'blue':
            self.p_r.set_off()
            self.p_b.set_on()
            self.p_g.set_off()
        elif color == 'white':
            self.p_r.set_on()
            self.p_b.set_on()
            self.p_g.set_on()
        elif color == 'violet':
            self.p_r.set_on()
            self.p_b.set_on()
            self.p_g.set_off()
        elif color == 'cyan':
            self.p_r.set_off()
            self.p_b.set_on()
            self.p_g.set_on()
        elif color == 'yellow':
            self.p_r.set_on()
            self.p_b.set_off()
            self.p_g.set_on()
        elif color == 'closed':
            self.p_r.set_off()
            self.p_b.set_off()
            self.p_g.set_off()


class Accelerometer(control_interfaces.AccelerometerInterface):
    '''
    Accelerometer
    Accelerometer(address)
    '''

    def __init__(self, address=0x68):
        self.sensor = mpu6050(address)

    def get_acceleration(self, dimension: str = "all") -> dict:
        accel = self.sensor.get_accel_data()
        if dimension == "all":
            return accel
        if dimension in ('x', 'y', 'z'):
            return accel[dimension]
        print("Dimension not recognized!!")
        return 0

    def get_gyro(self, dimension: str = "all") -> dict:
        gyro = self.sensor.get_gyro_data()
        if dimension == "all":
            return gyro
        if dimension in ('x', 'y', 'z'):
            return gyro[dimension]
        print("Dimension not recognized!!")
        return 0


#General functions
def start_lib():
    '''
    This function sets the GPIO pins to input output mode with GPIO number syntax.
    '''
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

def clean():
    '''
    This function releases all the GPIO pins .
    '''
    GPIO.cleanup()
