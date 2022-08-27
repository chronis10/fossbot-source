"""
Implementation of electronic parts control
"""

import math
import time
import RPi.GPIO as GPIO
from mpu6050 import mpu6050
import Adafruit_MCP3008
from common.interfaces import control_interfaces

class GenInput(control_interfaces.NoiseInterface):
    '''
    Class gen_input(pin)
    Default pin 4
    Functions:
    get_state(): Returns state 0 (False) or 1 (True)
    '''
    def __init__(self, pin: int = 4) -> None:
        self.pin = pin
        GPIO.setup(self.pin, GPIO.IN)

    def get_state(self) -> int:
        '''
        Returns state 0 or 1
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
    def __init__(self, pin: int = 5) -> None:
        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, False)

    def set_on(self) -> None:
        '''
        Set High the output pin
        '''
        GPIO.output(self.pin, True)

    def set_off(self) -> None:
        '''
        Set Low the output pin
        '''
        GPIO.output(self.pin, False)



class Motor(control_interfaces.MotorInterface):
    """
    Motor(speed_pin,terma_pin,termb_pinfreq=17,dc=70) -> Motor control.
    Functions:
    dir_control(direction) Change motor direction to input direction.
    move(direction) Start moving motor with default speed towards input direction.
    set_speed(speed) Set speed immediately 0-100% range.
    stop() Stops the motor.
    """

    def __init__(self, speed_pin: int, terma_pin: int, termb_pin: int, dc_value: int = 70) -> None:
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
        '''
        Set speed immediately 0-100% range.
        Param: speed: the range 0 - 100% that speed will be changed to.
        '''
        if speed < 0 or speed > 100:
            print(
                "The motor speed is a percentage of total motor power. Accepted values 0-100.")
        else:
            self.dc_value = speed
            self.mot.ChangeDutyCycle(speed)

    def dir_control(self, direction: str) -> None:
        '''
        Change motor direction to input direction.
        Param: direction: the direction to be headed to.
        '''
        if direction == "forward":
            GPIO.output(self.terma_pin, GPIO.HIGH)
            GPIO.output(self.termb_pin, GPIO.LOW)
        elif direction == "reverse":
            GPIO.output(self.terma_pin, GPIO.LOW)
            GPIO.output(self.termb_pin, GPIO.HIGH)
        else:
            print("Motor accepts only forward and reverse values")

    def move(self, direction: str = "forward") -> None:
        '''
        Start moving motor with default speed towards input direction.
        Param: direction: the direction to be headed to.
        '''
        self.dir_control(direction)
        self.mot.ChangeDutyCycle(self.dc_value)

    def stop(self) -> None:
        '''Stops the motor.'''
        self.mot.ChangeDutyCycle(0)


class Odometer(control_interfaces.OdometerInterface):
    '''
    Class Odometer(pin) -> Odometer control.
    (Event triggered counter).
    Functions:
    count_revolutions() Increases the counter of revolutions.
    get_revolutions() Returns the number of revolutions.
    get_distance() Returns the traveled distance in cm.
    reset() Resets the steps counter.
    '''
    def __init__(self, pin: int) -> None:
        self.pin = pin
        self.sensor_disc = 20
        self.steps = 0
        self.wheel_diameter = 6.65
        self.precision = 2
        GPIO.setup(pin, GPIO.IN)
        GPIO.add_event_detect(
            self.pin,
            GPIO.RISING,
            callback=self.count_revolutions,
            bouncetime=1)

    def count_revolutions(self) -> None:
        '''Increase total steps by one.'''
        self.steps += 1

    def get_steps(self) -> int:
        ''' Returns total number of steps. '''
        return self.steps

    def get_revolutions(self) -> float:
        ''' Returns total number of revolutions. '''
        return self.steps / self.sensor_disc

    def get_distance(self) -> float:
        ''' Return the total distance traveled so far (in cm). '''
        circumference = self.wheel_diameter * math.pi
        revolutions = self.steps / self.sensor_disc
        distance = revolutions * circumference
        return round(distance, self.precision)

    def reset(self) -> None:
        ''' Reset the total traveled distance and revolutions. '''
        self.steps = 0


class UltrasonicSensor(control_interfaces.UltrasonicSensorInterface):
    '''
    Class UltrasonicSensor(echo_pin,trig_pin) -> Ultrasonic sensor control.
    Functions:
    get_distance() return distance in cm.
    '''

    def __init__(self, echo_pin: int = 14, trig_pin: int = 15) -> None:
        self.echo_pin = echo_pin
        self.trig_pin = trig_pin
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.output(self.trig_pin, False)

    def get_distance(self) -> float:
        '''
        Gets the distance to the closest obstacle.
        Returns: the distance to the closest obstacle (in cm).
        '''
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
    Class AnalogueReadings(clk_p,miso_p,mosi_p,cs_p) -> Handles Analogue Readings.
    Functions:
    get_reading(pin) Gets reading of a specific sensor specified by input pin.
    '''

    def __init__(self, clk_p: int = 11, miso_p: int = 9, mosi_p: int = 10, cs_p: int = 8) -> None:
        self.mcp = Adafruit_MCP3008.MCP3008(clk=clk_p, cs=cs_p, miso=miso_p, mosi=mosi_p)

    def get_reading(self, pin: int) -> float:
        '''
        Gets reading of a specific sensor specified by input pin.
        Param: pin: the pin of the sensor.
        Returns: the reading of the requested sensor.
        '''
        value = self.mcp.read_adc(pin)
        print(f'ADC {pin}: {value}')
        return value


class LedRGB(control_interfaces.LedRGBInterface):
    '''
    Class LedRGB(pin_r,pin_b,pin_g) -> Led control
    set_on(color): sets led to input color.
    '''

    def __init__(self, pin_r: int = 16, pin_b: int = 19, pin_g: int = 12) -> None:
        self.p_r = GenOutput(pin_r)
        self.p_b = GenOutput(pin_b)
        self.p_g = GenOutput(pin_g)

    def set_on(self, color: str) -> None:
        '''
        Changes the color of a led
        Param: color: the wanted color
        For closing the led, use color == 'closed'
        '''
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
    Class Accelerometer(address) -> Handles accelerometer and gyroscope.
    Functions:
    get_acceleration(dimension) Returns the acceleration for a specific dimension.
    get_gyro(dimension) Returns the gyroscope for a specific dimension.
    '''

    #!FIXME what datatype is address (hexademical)?
    def __init__(self, address: int = 0x68) -> None:
        #hex(104) == 0x68
        self.sensor = mpu6050(address)

    def get_acceleration(self, dimension: str) -> float:
        '''
        Gets the acceleration for a specific dimension.
        Param: dimension: the dimension requested.
        Returns: the acceleration for a specific dimension.
        '''
        accel = self.sensor.get_accel_data()
        if dimension in ('x', 'y', 'z'):
            return accel[dimension]
        print("Dimension not recognized!!")
        return 0.0

    def get_gyro(self, dimension: str) -> float:
        '''
        Gets gyroscope for a specific dimension.
        Param: dimension: the dimension requested.
        Returns: the gyroscope for a specific dimension.
        '''
        gyro = self.sensor.get_gyro_data()
        if dimension in ('x', 'y', 'z'):
            return gyro[dimension]
        print("Dimension not recognized!!")
        return 0.0


#General functions
def start_lib() -> None:
    '''
    This function sets the GPIO pins to input output mode with GPIO number syntax.
    '''
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

def clean() -> None:
    '''
    This function releases all the GPIO pins .
    '''
    GPIO.cleanup()
