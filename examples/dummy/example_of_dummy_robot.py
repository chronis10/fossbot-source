""" Example of a dummy robot"""

import time
# from fossbot_lib.parameters_parser.parser import load_parameters
# from fossbot_lib.common.data_structures import configuration
from fossbot_lib.common.interfaces import robot_interface
from fossbot_lib.dummy_robot.fossbot import FossBot as DummyBot

def main(robot: robot_interface.FossBotInterface) -> None:
    """ A simple robot routine """
    robot.just_move()
    time.sleep(10)
    robot.stop()

if __name__ == "__main__":
    # Create a real robot
    dummy = DummyBot()
    main(dummy)
