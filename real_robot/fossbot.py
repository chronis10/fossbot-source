"""
Real robot implementation
"""

from common.data_structures import configuration
from common.interfaces import robot_interface
from real_robot import control


class FossBot(robot_interface.FossBotInterface):
    """ Real robot """

    def __init__(self,parameters: configuration.RobotParameters):
        self.motor_right  = control.Motor(speed_pin=23,terma_pin=27,termb_pin=22,
        dc_value=parameters.motor_right_speed.value)
        self.motor_left = control.Motor(speed_pin=25,terma_pin=17,termb_pin=24,
        dc_value=parameters.motor_right_speed.value)
        self.ultrasonic = control.UltrasonicSensor(echo_pin=5,trig_pin=6)
        self.parameters = parameters

    def get_distance(self) -> float:
        """ Return Distance in cm  """
        return self.ultrasonic.get_distance()

    def check_for_obstacle(self) -> bool:
        """ Return if obstacle detected """
        if self.ultrasonic.get_distance() <= self.parameters.sensor_distance.value:
            return True
        return False

    def just_move(self, direction: str = "forward") -> None:
        """ Move forward forever """
        self.motor_left.move(direction =direction)
        self.motor_right.move(direction =direction)

    def stop(self) -> None:
        """ Stop moving """
        self.motor_left.stop()
        self.motor_right.stop()
