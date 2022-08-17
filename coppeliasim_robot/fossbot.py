"""
Real robot implementation
"""

from common.data_structures import configuration
from common.interfaces import robot_interface
from coppeliasim_robot import control


class FossBot(robot_interface.FossBotInterface):
    """ Real robot """

    def __init__(self, parameters: configuration.RobotParameters):
        self.motor_right  = control.Motor(motor_joint_name="something")
        self.motor_left = control.Motor(motor_joint_name="something")
        self.ultrasonic = control.UltrasonicSensor()
        self.parameters = parameters
        raise NotImplementedError

    def get_distance(self) -> float:
        """ Return Distance in cm  """
        raise NotImplementedError

    def check_for_obstacle(self) -> bool:
        """ Return if obstacle detected """
        raise NotImplementedError

    def just_move(self, direction: str = "forward") -> None:
        """ Move forward forever """
        raise NotImplementedError

    def stop(self) -> None:
        """ Stop moving """
        raise NotImplementedError
