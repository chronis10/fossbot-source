"""
Class for handling godot connections (get and post requests to socketio server).
"""
import socketio
import threading
import time

class GodotHandler():
    """ Godot Handler """
    def __init__(self, socket: socketio.Client, fossbot_name: str, socketio_namespace: str = "/godot"):
        self.response = None
        self.fossbot_name = fossbot_name
        self.socketio_namespace = socketio_namespace
        self.event = threading.Event()
        self.socket = socket

        @self.socket.on("godotMessage", namespace=self.socketio_namespace)
        def godotMessage(response):
            self.response = response
            self.event.set()  # signal that the event has occurred

        @self.socket.on("godotError", namespace=self.socketio_namespace)
        def godotError(response):
            self.event.set()
            self.socket.disconnect()
            raise ConnectionError(response["data"])

    def post_godot(self, param: dict):
        '''
        Used to post a response from godot (POST).
        Param: param: the dictionary to be sent to godot.
        '''
        param["fossbot_name"] = self.fossbot_name
        self.socket.emit("clientMessage", param, namespace=self.socketio_namespace)
        time.sleep(0.1)

    def post_godot_env(self, param: dict):
        '''
        Used to post a response from godot (POST).
        Param: param: the dictionary to be sent to godot.
        '''
        self.socket.emit("clientMessage", param, namespace=self.socketio_namespace)
        time.sleep(0.1)

    def get_godot(self, param: dict):
        '''
        Used to get a response from godot (GET).
        Param: param: the dictionary to be sent to godot (to get the data).
        Returns: the response of the godot sim.
        '''
        self.post_godot(param)
        self.event.wait()  # wait for the event to be set
        self.event.clear()  # clear the event for the next use
        time.sleep(0.1)
        return self.response["data"]
