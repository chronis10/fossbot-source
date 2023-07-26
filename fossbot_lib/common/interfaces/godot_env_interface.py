"""
Interface for Godot Environment.
"""

from abc import ABC, abstractmethod

class GodotEnvInterface(ABC):
    """ Interface for Godot Environment. """

    @abstractmethod
    def spawn_fossbot(self, **kwargs) -> None:
        """
        Spawns a Fossbot in the Godot simulator with the specified parameters.
        Parameters:
            kwargs (optional):
                - pos_x (float): The X coordinate of the spawn position. Defaults to 1.
                - pos_y (float): The Y coordinate of the spawn position. Defaults to 1.
                - pos_z (float): The Z coordinate of the spawn position. Defaults to 0.
                - color (str): The color of the Fossbot. Defaults to "blue".
                - rotation (float): The initial rotation of the Fossbot (in degrees). Defaults to 0.
                - counterclockwise (bool): Whether the rotation parameter will be counterclockwise. Defaults to False.
        """

    @abstractmethod
    def spawn_cube(self, **kwargs) -> None:
        """
        Spawns a cube in the Godot simulator with the specified parameters.

        Parameters:
            kwargs (optional):
                - pos_x (float): The X coordinate of the spawn position. Defaults to 1.
                - pos_y (float): The Y coordinate of the spawn position. Defaults to 1.
                - pos_z (float): The Z coordinate of the spawn position. Defaults to 0.
                - scale_x (float): The scale of the cube along the X-axis. Defaults to 1.
                - scale_y (float): The scale of the cube along the Y-axis. Defaults to 1.
                - scale_z (float): The scale of the cube along the Z-axis. Defaults to 1.
                - color (str): The color of the cube. Defaults to "white".
                - rotation (float): The initial rotation of the cube (in degrees). Defaults to 0.
                - counterclockwise (bool): Whether the rotation parameter will be counterclockwise. Defaults to False.
        """

    @abstractmethod
    def spawn_sphere(self, **kwargs) -> None:
        """
        Spawns a sphere in the Godot simulator with the specified parameters.

        Parameters:
            kwargs (optional):
                - pos_x (float): The X coordinate of the spawn position. Defaults to 1.
                - pos_y (float): The Y coordinate of the spawn position. Defaults to 1.
                - pos_z (float): The Z coordinate of the spawn position. Defaults to 0.
                - color (str): The color of the sphere. Defaults to "white".
                - radius (float): The radius of the sphere. Defaults to 1.
        """

    @abstractmethod
    def draw_image_floor(self, image_path: str, **kwargs) -> None:
        """
        Changes the appearance of the floor in the Godot simulator by drawing an image (manually).

        Parameters:
            image_path (str): The path to the image file in your pc.

        Optional Parameters:
            - floor_index (int): The index of the floor to apply the image to. Defaults to 0.
            - color (str): The color of the image. Defaults to "white".
            - tripl (bool): If True, the image is drawn using "triplanar" method, otherwise "manual" method is used. Defaults to False.
            - scale_x (float): The scale factor along the X-axis for the image. Defaults to 1.
            - scale_y (float): The scale factor along the Y-axis for the image. Defaults to 1.
        """

    @abstractmethod
    def draw_image_floor_auto(self, image_path: str, **kwargs) -> None:
        """
        Changes the appearance of the floor in the Godot simulator by drawing an image and automatically scaling it.

        Parameters:
            image_path (str): The path to the image file in your pc.

        Optional Parameters:
            - floor_index (int): The index of the floor to apply the image to. Defaults to 0.
            - color (str): The color of the image. Defaults to "white".
        """

    @abstractmethod
    def change_floor_terrain(self, image_path: str, **kwargs) -> None:
        """
        BETA VERSION: Changes the ground terrain in the Godot simulator based on the specified height map.

        Parameters:
            image_path (str): The path to the height map image file in your pc.

        Optional Parameters:
            - floor_index (int): The index of the floor to apply the terrain to. Defaults to 0.
            - intensity (int): The intensity of the terrain deformation. Defaults to 3.
        """

    # exit
    @abstractmethod
    def exit(self) -> None:
        ''' Exit for the environment user. '''
