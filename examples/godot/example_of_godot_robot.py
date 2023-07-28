""" Example of a godot robot"""

import os
import time
import pathlib
from threading import Thread
from fossbot_lib.common.interfaces import robot_interface
from fossbot_lib.godot_robot.fossbot import FossBot
from fossbot_lib.godot_robot.godot_env import GodotEnvironment

def main(robot: robot_interface.FossBotInterface) -> None:
    """ A complex robot routine """
    robot.move_distance(15, "reverse")
    while True:
        robot.just_move()
        if robot.get_distance() < 5:
            robot.stop()
            break
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

def level_0(session_id) -> None:
    """ A simple level to showcase fossbot's functions. """
    ge = GodotEnvironment(session_id)
    ge.remove_all_objects()
    ge.spawn_fossbot(pos_x=0, pos_y=0)
    ge.spawn_cube(pos_x=-10, pos_y=0)
    ge.exit()
    f = FossBot(session_id)
    main(f)

def level_1(session_id: str) -> None:
    """ Creates the stage for level 1. """
    ge = GodotEnvironment(session_id)
    ge.remove_all_objects()
    ge.spawn_fossbot(pos_x=11, pos_y=2, rotation=90, counterclockwise=True)
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
    # example solution:
    r = FossBot(session_id, fossbot_name="fossbot")
    while True:
        d = r.get_distance()
        print(d)
        if d > 5:
            r.just_move()
        else:
            r.rotate_clockwise_90()

def level_2(session_id):
    """ Generates a circle (stored as a jpg) on the floor. """
    ge = GodotEnvironment(session_id)
    ge.remove_all_objects()
    current_path = pathlib.Path(__file__).parent.resolve()
    path_dir = os.path.join(current_path, 'images')
    file_path = os.path.join(path_dir, 'circle.jpg')
    ge.draw_image_floor_auto(file_path)
    ge.spawn_fossbot(pos_x=44.5, pos_y=1, rotation=90, counterclockwise=True)
    ge.exit()
    robot = FossBot(session_id, fossbot_name="fossbot")
    # Follows black line. (example solution)
    while True:
        middle = robot.check_on_line(1)
        right = robot.check_on_line(2)
        left = robot.check_on_line(3)
        print(left, middle, right)
        if middle and right and left:
            robot.move_forward()
        elif middle and left and not right:
            robot.rotate_counterclockwise()
        elif middle and right and not left:
            robot.rotate_clockwise()
        elif left:
            robot.rotate_counterclockwise()
        elif right:
            robot.rotate_clockwise()
        else:
            print("Exited circle.")
            break
    robot.exit()

def level_3(session_id):
    """ Showcases multiple fossbot control. """
    ge = GodotEnvironment(session_id)
    ge.remove_all_objects()
    ge.spawn_sphere(pos_x=0, pos_y=0, radius=2, color="green")
    ge.spawn_fossbot(pos_y=0, pos_x=20)
    ge.spawn_fossbot(pos_y=-30, pos_x=0, rotation=90, color="red")
    ge.spawn_fossbot(pos_y=0, pos_x=-10, rotation=180, color="yellow")
    ge.spawn_fossbot(pos_y=15, pos_x=0, rotation=90, counterclockwise=True, color="cyan")
    ge.exit()
    def __foss_main(session_id, fossbot_name):
        r = FossBot(session_id, fossbot_name=fossbot_name)
        main(r)
    thread = Thread(target = __foss_main, args = (session_id, "fossbot2"))
    thread.start()
    time.sleep(0.1)
    thread = Thread(target = __foss_main, args = (session_id, "fossbot3"))
    thread.start()
    time.sleep(0.1)
    thread = Thread(target = __foss_main, args = (session_id, "fossbot4"))
    thread.start()
    time.sleep(0.1)
    __foss_main(session_id, "fossbot")

def level_4(session_id):
    """ Generates and navigates terrain (BETA VERSION). """
    ge = GodotEnvironment(session_id)
    current_path = pathlib.Path(__file__).parent.resolve()
    path_dir = os.path.join(current_path, 'images')
    heightmap_img = os.path.join(path_dir, 'heightmap.jpg')
    grass_img = os.path.join(path_dir, 'grass.png')
    ge.change_floor_terrain(heightmap_img, intensity=20)
    ge.draw_image_floor_auto(grass_img)
    ge.spawn_fossbot(pos_x=-30, pos_y=30, rotation=110, counterclockwise=True)
    ge.exit()

    # Example solution (navigation):
    f = FossBot(session_id, rotate_90=40)
    right_dist = 0
    left_dist = 0
    doOnce = True
    count_times = 0
    while count_times <= 4:
        d = f.get_distance()
        if d < 5:
            if doOnce:
                count_times += 1
                f.rotate_clockwise_90()
                right_dist = f.get_distance()
                if right_dist > d:
                    doOnce = False
                    continue
                f.rotate_counterclockwise_90()
                f.rotate_counterclockwise_90()
                left_dist = f.get_distance()
                # f.rotate_clockwise_90()
                doOnce = False
            if right_dist < left_dist:
                f.rotate_counterclockwise()
            else:
                f.rotate_clockwise()
        else:
            doOnce = True
            f.just_move()
    f.exit()

if __name__ == "__main__":
    # Create a real robot
    godot = FossBot(session_id="8f61695b-0d64-4e67-9887-3ec1a69905b1", fossbot_name="fossbot")
    main(godot)
