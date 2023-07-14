""" Example of a godot robot"""

from fossbot_lib.common.interfaces import robot_interface
from fossbot_lib.godot_robot.fossbot import FossBot as GodotBot

def main(robot: robot_interface.FossBotInterface) -> None:
    """ A simple robot routine """
    robot.move_distance(15, "reverse")
    while True:
        robot.just_move()
        if robot.get_distance() < 3:
            robot.stop()
            break
    for i in range(0, 5):
        robot.just_rotate(0)
    robot.just_rotate(1)
    for i in range(0, 5):
        robot.just_move("reverse")
    robot.rotate_90(0)
    print("rotated.")
    print(robot.check_for_dark())
    robot.play_sound("res://soundfx/startup.mp3")
    robot.move_distance(3, "reverse")
    print("distance travelled")
    robot.rotate_90(1)
    print("rotated.")
    print(robot.get_distance())
    print(robot.check_for_obstacle())
    print(robot.get_acceleration('x'))
    print(robot.get_gyroscope('z'))
    robot.rgb_set_color("red")
    robot.move_distance(4)
    robot.wait(1)
    robot.rotate_clockwise_90()
    robot.rotate_counterclockwise_90()
    robot.wait(1)
    robot.exit()

if __name__ == "__main__":
    # Create a real robot
    godot = GodotBot(session_id="8f61695b-0d64-4e67-9887-3ec1a69905b1")
    main(godot)
