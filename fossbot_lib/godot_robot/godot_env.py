import base64
import socketio
from godot_handler import GodotHandler

class GodotEnvironment():
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
            self.sio.emit('pythonConnect', {"session_id": self.session_id, "user_id" :self.sio.get_sid(namespace=namespace)}, namespace=namespace)
            self.godotHandler = GodotHandler(self.sio, "", namespace)
            print(f"Connected to socketio server on {server_address}")


        server_address = kwargs.get("server_address", 'http://localhost:8000')

        self.sio.connect(server_address + namespace, namespaces=[namespace])


    def spawn_fossbot(self, **kwargs) -> None:
        param = {
            "func": "foss_spawn",
            "pos_x":kwargs.get("pos_x", 1),
            "pos_y":kwargs.get("pos_y", 1),
            "pos_z":kwargs.get("pos_z", 0),
            "color":kwargs.get("color", "blue")
        }
        self.godotHandler.post_godot_env(param)

    def spawn_cube(self, **kwargs):
        param = {
            "func": "obs_spawn",
            "pos_x":kwargs.get("pos_x", 1),
            "pos_y":kwargs.get("pos_y", 1),
            "scale_x":kwargs.get("scale_x", 1),
            "scale_y":kwargs.get("scale_y", 1),
            "scale_z":kwargs.get("scale_z", 1),
            "type":kwargs.get("type", "cube"),
            "pos_z":kwargs.get("pos_z", 0),
            "color":kwargs.get("color", "white"),
        }
        self.godotHandler.post_godot_env(param)

    def spawn_sphere(self, **kwargs):
        param = {
            "func": "obs_spawn",
            "pos_x":kwargs.get("pos_x", 1),
            "pos_y":kwargs.get("pos_y", 1),
            "type":kwargs.get("type", "sphere"),
            "pos_z":kwargs.get("pos_z", 0),
            "color":kwargs.get("color", "white"),
            "radius":kwargs.get("radius", 1)
        }
        self.godotHandler.post_godot_env(param)


    def __send_chunk_image(self, req_func: str, image_path: str, chunk_size: int = 20000):
        """
        Sends the image as chunks.
        Param:
         - req_func: the requested function for image load in godot.
         - image_path: the path to the image.
         - chunk_size: the size of each chunk sent to the server. Change it if necessary to match your needs.
        Returns: the number of chunks (so it can be used later).
        """
        with open(image_path, "rb") as f:
            image_data = f.read()
        base64_image_str = base64.b64encode(image_data).decode()
        chunks = [base64_image_str[i:i + chunk_size] for i in range(0, len(base64_image_str), chunk_size)]

        for chunk in chunks:
            param = {
                "func": req_func,
                "image": chunk,
                "image_size": len(chunks)
            }
            self.godotHandler.post_godot_env(param)
        return len(chunks)


    def draw_image_floor(self, image_path: str, **kwargs) -> None:
        '''
        Changes the path of the scene according to the image.
        '''

        chunk_size = self.__send_chunk_image("change_floor_skin", image_path)

        draw_type = "manual"
        if bool(kwargs.get("tripl", False)):
            draw_type = "tripl"
        param = {
            "func": "change_floor_skin",
            "floor_index": str(kwargs.get("floor_index", 0)),
            "image_size": chunk_size,
            "color": kwargs.get("color", "white"),
            "type": draw_type,
            "scale_x":kwargs.get("scale_x", 1),
            "scale_y":kwargs.get("scale_y", 1)
        }
        self.godotHandler.post_godot_env(param)


    def draw_image_floor_auto(self, image_path: str, **kwargs) -> None:
        '''
        Changes the path (and scales it automatically) of the scene according to the image.
        '''

        chunk_size = self.__send_chunk_image("change_floor_skin", image_path)

        param = {
            "func": "change_floor_skin",
            "floor_index": str(kwargs.get("floor_index", 0)),
            "image_size": chunk_size,
            "color": kwargs.get("color", "white"),
            "type": "full"
        }
        self.godotHandler.post_godot_env(param)


    def load_sim_image_floor(self, **kwargs) -> None:
        """
        Load a floor skin preset from the simulator.
        """
        param = {
            "func": "load_sim_image_floor",
            "floor_index": str(kwargs.get("floor_index", 0)),
            # "color": kwargs.get("color", "white")
        }
        self.godotHandler.post_godot_env(param)

    # exit
    def exit(self) -> None:
        ''' Exits. '''
        if self.sio.connected:
            self.godotHandler.post_godot_env({"func":"exit", "env_user":True})
            self.sio.disconnect()

    def __del__(self) -> None:
        self.exit()
        print('Program ended.')
