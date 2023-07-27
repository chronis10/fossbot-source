"""
Implementation for godot environment.
"""
import base64
import socketio
from fossbot_lib.common.interfaces import godot_env_interface
from fossbot_lib.godot_robot.godot_handler import GodotHandler


class GodotEnvironment(godot_env_interface.GodotEnvInterface):
    """ Godot robot """

    def __init__(self, session_id: str, **kwargs) -> None:
        """
        Initializes a Godot Environment Object with the provided session ID and optional parameters.
        Param:
            session_id (str): The session ID of the Godot simulator in the browser.
            kwargs (optional):
             - server_address (str): The address of the server. Defaults to 'http://localhost:8000'.
             - namespace (str): The namespace of the socketio for fossbot sim (default is "/godot").
        """
        self.session_id = session_id
        self.sio = socketio.Client()

        namespace = kwargs.get("namespace", "/godot")

        @self.sio.event(namespace=namespace)
        def connect():
            self.sio.emit('pythonConnect', {"session_id": self.session_id, "user_id" :self.sio.get_sid(namespace=namespace), "env_user": True, "func":"connect_env"}, namespace=namespace)
            self.godotHandler = GodotHandler(self.sio, "", namespace)
            print(f"Connected to socketio server on {server_address}")


        server_address = kwargs.get("server_address", 'http://localhost:8000')

        self.sio.connect(server_address + namespace, namespaces=[namespace])


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
        param = {
            "func": "foss_spawn",
            "pos_x":float(kwargs.get("pos_x", 1)),
            "pos_y":float(kwargs.get("pos_y", 1)),
            "pos_z":float(kwargs.get("pos_z", 0)),
            "color":kwargs.get("color", "blue"),
            "rotation":float(kwargs.get("rotation", 0)),
            "counterclockwise":bool(kwargs.get("counterclockwise", False))
        }
        self.godotHandler.post_godot_env(param)

    def __check_scale(self, kwargs, incl_scale_z: bool = True):
        scale_x = float(kwargs.get("scale_x", 1))
        if scale_x <= 0:
            print("Invalid scale x.")
            scale_x = 1
        scale_y = float(kwargs.get("scale_y", 1))
        if scale_y <= 0:
            print("Invalid scale y.")
            scale_y = 1
        scale_z = float(kwargs.get("scale_z", 1))
        if scale_z <= 0:
            if not incl_scale_z:
                print("Invalid scale z.")
            scale_z = 1
        return scale_x, scale_y, scale_z

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
        scale_x, scale_y, scale_z = self.__check_scale(kwargs)
        param = {
            "func": "obs_spawn",
            "pos_x":float(kwargs.get("pos_x", 1)),
            "pos_y":float(kwargs.get("pos_y", 1)),
            "scale_x":scale_x,
            "scale_y":scale_y,
            "scale_z":scale_z,
            "type":"cube",
            "pos_z":float(kwargs.get("pos_z", 0)),
            "color":kwargs.get("color", "white"),
            "rotation":float(kwargs.get("rotation", 0)),
            "counterclockwise":bool(kwargs.get("counterclockwise", False))
        }
        self.godotHandler.post_godot_env(param)


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
        radius = float(kwargs.get("radius", 1))
        if radius <= 0:
            print("Invalid radius.")
            radius = 1
        param = {
            "func": "obs_spawn",
            "pos_x":float(kwargs.get("pos_x", 1)),
            "pos_y":float(kwargs.get("pos_y", 1)),
            "type":"sphere",
            "pos_z":float(kwargs.get("pos_z", 0)),
            "color":kwargs.get("color", "white"),
            "radius":radius
        }
        self.godotHandler.post_godot_env(param)


    def spawn_cone(self, **kwargs) -> None:

        """
        Spawns a cone in the Godot simulator with the specified parameters.

        Parameters:
            kwargs (optional):
             - pos_x (float): The X coordinate of the spawn position. Defaults to 1.
             - pos_y (float): The Y coordinate of the spawn position. Defaults to 1.
             - pos_z (float): The Z coordinate of the spawn position. Defaults to 0.
             - scale_x (float): The scale of the cone along the X-axis. Defaults to 1.
             - scale_y (float): The scale of the cone along the Y-axis. Defaults to 1.
             - scale_z (float): The scale of the cone along the Z-axis. Defaults to 1.
             - color (str): The color of the cone. Defaults to "white".
             - rotation (float): The initial rotation of the cone (in degrees). Defaults to 0.
             - counterclockwise (bool): Whether the rotation parameter will be counterclockwise. Defaults to False.
        """
        scale_x, scale_y, scale_z = self.__check_scale(kwargs)
        param = {
            "func": "obs_spawn",
            "pos_x":float(kwargs.get("pos_x", 1)),
            "pos_y":float(kwargs.get("pos_y", 1)),
            "scale_x":scale_x,
            "scale_y":scale_y,
            "scale_z":scale_z,
            "type":"cone",
            "pos_z":float(kwargs.get("pos_z", 0)),
            "color":kwargs.get("color", "white"),
            "rotation":float(kwargs.get("rotation", 0)),
            "counterclockwise":bool(kwargs.get("counterclockwise", False))
        }
        self.godotHandler.post_godot_env(param)


    def change_fossbot(self, fossbot_name: str = "fossbot", **kwargs) -> None:
        """
        Changes an existing Fossbot's characteristics (like position, rotation or color) in the Godot simulator.
        Parameters:
            fossbot_name (str): The name of the fossbot you want to change in the scene. Defaults to "fossbot".
            kwargs (optional):
             - pos_x (float): The X coordinate of the position to move the fossbot. Defaults to 1.
             - pos_y (float): The Y coordinate of the position to move the fossbot. Defaults to 1.
             - pos_z (float): The Z coordinate of the position to move the fossbot. Defaults to 0.
             - color (str): The color of the Fossbot. Defaults to "blue".
             - rotation (float): The rotation of the Fossbot (in degrees). Defaults to 0.
             - counterclockwise (bool): Whether the rotation parameter will be counterclockwise. Defaults to False.
        """
        param = {
            "func": "change_fossbot",
            "fossbot_name":fossbot_name,
            "pos_x":float(kwargs.get("pos_x", 1)),
            "pos_y":float(kwargs.get("pos_y", 1)),
            "pos_z":float(kwargs.get("pos_z", 0)),
            "color":kwargs.get("color", "blue"),
            "rotation":float(kwargs.get("rotation", 0)),
            "counterclockwise":bool(kwargs.get("counterclockwise", False))
        }
        self.godotHandler.post_godot_env(param)


    def change_floor_size(self, **kwargs) -> None:
        """
        Changes the size of the floor in the Godot simulator. Make sure to change fossbot position afterwards,
        or spawn objects (so they dont fall of the scene).

        Optional Parameters:
         - floor_index (int): The index of the floor to apply the image to. Defaults to 0.
         - scale_x (float): The scale in x axis of the floor. If not specified, keeps the current floor x scale.
         - scale_y (float): The scale in y axis of the floor. If not specified, keeps the current floor y scale.
        """
        param = {
            "func": "change_floor",
            "floor_index": str(int(kwargs.get("floor_index", 0))),
        }

        scale_x, scale_y, _ = self.__check_scale(kwargs, False)
        param["scale_x"] = scale_x
        param["scale_y"] = scale_y

        self.godotHandler.post_godot_env(param)


    def __send_chunk_image(self, req_func: str, image_path: str, chunk_size: int = 20000) -> int:
        """
        Sends the image as chunks.
        Param:
         - req_func: the requested function for image load in godot.
         - image_path: the path to the image in your pc.
         - chunk_size: the size of each chunk sent to the server. Change it if necessary to match your needs.
        Returns: the number of chunks (so it can be used later).
        """
        with open(image_path, "rb") as f:
            image_data = f.read()
        base64_image_str = base64.b64encode(image_data).decode()
        chunks = [base64_image_str[i:i + chunk_size] for i in range(0, len(base64_image_str), chunk_size)]

        for i, chunk in enumerate(chunks):
            param = {
                "func": req_func,
                "image": chunk,
                "image_size": len(chunks),
                "img_num": i
            }
            self.godotHandler.post_godot_env(param)
        return len(chunks)


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

        chunk_size = self.__send_chunk_image("change_floor_skin", image_path)
        scale_x, scale_y, _ = self.__check_scale(kwargs, False)
        draw_type = "manual"
        if bool(kwargs.get("tripl", False)):
            draw_type = "tripl"
        param = {
            "func": "change_floor_skin",
            "floor_index": str(int(kwargs.get("floor_index", 0))),
            "image_size": chunk_size,
            "color": kwargs.get("color", "white"),
            "type": draw_type,
            "scale_x":scale_x,
            "scale_y":scale_y
        }
        self.godotHandler.get_godot_env(param)


    def draw_image_floor_auto(self, image_path: str, **kwargs) -> None:
        """
        Changes the appearance of the floor in the Godot simulator by drawing an image and automatically scaling it.

        Parameters:
            image_path (str): The path to the image file in your pc.

        Optional Parameters:
         - floor_index (int): The index of the floor to apply the image to. Defaults to 0.
         - color (str): The color of the image. Defaults to "white".
        """

        chunk_size = self.__send_chunk_image("change_floor_skin", image_path)

        param = {
            "func": "change_floor_skin",
            "floor_index": str(int(kwargs.get("floor_index", 0))),
            "image_size": chunk_size,
            "color": kwargs.get("color", "white"),
            "type": "full"
        }
        self.godotHandler.get_godot_env(param)


    def change_floor_terrain(self, image_path: str, **kwargs) -> None:
        """
        BETA VERSION: Changes the ground terrain in the Godot simulator based on the specified height map.

        Parameters:
            image_path (str): The path to the height map image file in your pc.

        Optional Parameters:
         - floor_index (int): The index of the floor to apply the terrain to. Defaults to 0.
         - intensity (float): The intensity of the terrain deformation. Defaults to 3.
        """

        chunk_size = self.__send_chunk_image("change_floor_terrain", image_path)

        param = {
            "func": "change_floor_terrain",
            "floor_index": str(int(kwargs.get("floor_index", 0))),
            "image_size": chunk_size,
            "intensity": float(kwargs.get("intensity", 3)),
        }
        self.godotHandler.get_godot_env(param)


    # exit
    def exit(self) -> None:
        ''' Exit for the environment user. '''
        if self.sio.connected:
            # self.godotHandler.post_godot_env({"func":"exit_env"})
            self.sio.disconnect()


    def __del__(self) -> None:
        self.exit()
        print('Program ended.')
