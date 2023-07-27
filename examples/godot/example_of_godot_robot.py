""" Example of a godot robot"""

from fossbot_lib.common.interfaces import robot_interface
from fossbot_lib.godot_robot.fossbot import FossBot as GodotBot
from fossbot_lib.godot_robot.godot_env import GodotEnvironment

def main(robot: robot_interface.FossBotInterface) -> None:
    """ A complex robot routine """
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

def level_1(session_id: str) -> None:
    """ Creates the stage for level 1. """
    ge = GodotEnvironment(session_id)
    ge.change_fossbot(fossbot_name="fossbot", pos_x=11, pos_y=2, rotation=90, counterclockwise=True)
    ge.spawn_cube(pos_x=20.6, pos_y=-2.36, scale_y=20, scale_x=0.8)
    ge.spawn_cube(pos_y=-24, pos_x=1.62, rotation=90, scale_y=20, scale_x=0.8)
    ge.spawn_cube(pos_y=12, pos_x=0, scale_y=20, scale_x=0.8)
    ge.spawn_cube(pos_y=-2, pos_x=-17, scale_y=20, scale_x=0.8)
    ge.spawn_cube(pos_y=34, pos_x=3, rotation=90, scale_y=20, scale_x=0.8)
    ge.spawn_cube(pos_y=47, pos_x=21, rotation=90, scale_y=20, scale_x=0.8)
    ge.spawn_cube(pos_y=47, pos_x=-21, rotation=90, scale_y=20, scale_x=0.8)
    ge.spawn_cube(pos_y=27, pos_x=-43, scale_y=20, scale_x=0.8)
    ge.spawn_cube(pos_y=-15, pos_x=-43, scale_y=20, scale_x=0.8)
    ge.spawn_cube(pos_y=-40, pos_x=-24, rotation=90, scale_y=20, scale_x=0.8)
    ge.spawn_cube(pos_y=-40, pos_x=18, rotation=90, scale_y=20, scale_x=0.8)
    ge.spawn_cube(pos_y=-17, pos_x=38, scale_y=20, scale_x=0.8)
    ge.spawn_cube(pos_y=25, pos_x=38, scale_y=20, scale_x=0.8)
    ge.exit()

if __name__ == "__main__":
    # Create a real robot
    godot = GodotBot(session_id="8f61695b-0d64-4e67-9887-3ec1a69905b1", fossbot_name="fossbot")
    main(godot)
