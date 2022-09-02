'''Gym for simulated environment.'''

from abc import ABC, abstractmethod

# used only in simulation robot:
class EnvironmentInterface(ABC):
    """
    Interface for Environment control.
    Functions:
    draw_path(file_name,scale_x,scale_y) Changes the path of the scene.
    draw_path_auto(file_name) Changes the path of the scene and scales
                              it automatically on the floor.
    clear_path(): Clears the path of the scene.
    change_brightness(brightness): Changes scene's brightness.
    default_brightness(): Sets scene's brightness to default brightness (50%).
    get_simulation_time(): Returns current time of simulation.
    """

    # change path functions:
    @abstractmethod
    def draw_path(self, file_name: str, scale_x: float = 5.0, scale_y: float = 5.0) -> None:
        '''
        Changes the path of the scene.
        Param: file_name: the name of the picture to change the path to
               (save picture-path in paths folder).
               scale_x: scale x for image on the floor.
               scale_y: scale y for image on the floor.
        '''

    @abstractmethod
    def draw_path_auto(self, file_name: str) -> None:
        '''
        Changes the path of the scene and scales it automatically on the floor.
        Param: file_name: the name of the picture to change the path to
               (save picture-path in paths folder).
        '''

    @abstractmethod
    def clear_path(self) -> None:
        '''
        Clears the path of the scene.
        '''

    @abstractmethod
    def change_brightness(self, brightness: int = 50) -> None:
        '''
        Changes scene's brightness.
        Param: brightness: the percentage of the brightness to be changed to
               (default brightness == 50%).
        '''

    @abstractmethod
    def default_brightness(self) -> None:
        '''Sets scene's brightness to default brightness (50%).'''

    @abstractmethod
    def get_simulation_time(self) -> float:
        '''Returns current time of simulation.'''
