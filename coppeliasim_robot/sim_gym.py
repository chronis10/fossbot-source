import os
from coppeliasim_robot import control
from common.interfaces import sim_gym_interface
from common.data_structures import configuration
from coppeliasim_robot.fossbot import connect_vrep, sim

# used only in simulation robot:
class Environment(sim_gym_interface.EnvironmentInterface):
    """
    EnvironmentHandler(sim_param) -> Environment control.
    Functions:
    draw_path(file_name,scale_x,scale_y) Changes the path of the scene.
    draw_path_auto(file_name) Changes the path of the scene and scales
                              it automatically on the floor.
    clear_path(): Clears the path of the scene.
    change_brightness(brightness): Changes scene's brightness.
    default_brightness(): Sets scene's brightness to default brightness (50%).
    get_simulation_time(): Returns current time of simulation.
    """
    def __init__(self, parameters: configuration.SimRobotParameters) -> None:
        self.parameters = parameters
        # connects to server if not already connected:
        if self.parameters.simulation.client_id == None:
            self.client_id = connect_vrep()
            if self.client_id == -1:
                print('Failed connecting to remote API server')
                raise ConnectionError
            print('Connected to remote API server')
            self.parameters.simulation.client_id = self.client_id
        else:
            self.client_id = self.parameters.simulation.client_id

    # change path functions:
    def draw_path(self, file_name: str, scale_x: float = 5.0, scale_y: float = 5.0) -> None:
        '''
        Changes the path of the scene.
        Param: file_name: the name of the picture to change the path to
               (save picture-path in paths folder).
               scale_x: scale x for image on the floor.
               scale_y: scale y for image on the floor.
        '''
        path_dir_b = os.path.join(os.path.dirname(__file__), 'paths')
        path_draw = os.path.join(path_dir_b, file_name)
        if not os.path.exists(path_draw):
            print('Cannot find requested image.')
            raise FileNotFoundError
        while True:
            res, _, _, _, _ = control.exec_vrep_script(
                self.client_id, self.parameters.simulation.floor_name, 'draw_path',
                in_floats=[scale_x, scale_y], in_strings=[path_draw])
            if res == sim.simx_return_ok:
                break

    def draw_path_auto(self, file_name: str) -> None:
        '''
        Changes the path of the scene and scales it automatically on the floor.
        Param: file_name: the name of the picture to change the path to
               (save picture-path in paths folder).
        '''
        path_dir_b = os.path.join(os.path.dirname(__file__), 'paths')
        path_draw = os.path.join(path_dir_b, file_name)
        if not os.path.exists(path_draw):
            print('Cannot find requested image.')
            raise FileNotFoundError
        while True:
            res, _, _, _, _ = control.exec_vrep_script(
                self.client_id, self.parameters.simulation.floor_name, 'draw_path_auto',
                in_strings=[path_draw])
            if res == sim.simx_return_ok:
                break

    def clear_path(self) -> None:
        '''
        Clears the path of the scene.
        '''
        while True:
            res, _, _, _, _ = control.exec_vrep_script(
                self.client_id, self.parameters.simulation.floor_name,
                'clear_path')
            if res == sim.simx_return_ok:
                break

    def change_brightness(self, brightness: int = 50) -> None:
        '''
        Changes scene's brightness.
        Param: brightness: the percentage of the brightness to be changed to
               (default brightness == 50%).
        '''
        if brightness < 0 or brightness > 100:
            print("The brightness is a percentage. Accepted values 0-100.")
        else:
            print('Changing brightness...')
            brightness = brightness / 100
            while True:
                res, _, _, _, _ = control.exec_vrep_script(
                    self.client_id, self.parameters.simulation.floor_name,
                    'change_brightness', in_floats=[brightness, brightness, brightness])
                if res == sim.simx_return_ok:
                    break

    def default_brightness(self) -> None:
        '''Sets scene's brightness to default brightness (50%).'''
        self.change_brightness(50)

    def get_simulation_time(self) -> float:
        '''Returns current time of simulation.'''
        while True:
            res, _, sim_time, _, _ = control.exec_vrep_script(
                self.client_id, self.parameters.simulation.def_camera_name,
                'get_sim_time')
            if res == sim.simx_return_ok:
                return sim_time[0]
