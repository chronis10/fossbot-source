"""
Class for handling godot connections (get and post requests to socketio server).
"""
import socketio
import threading
import time

class GodotHandler():
    """ Godot Handler """
    def __init__(self, socket: socketio.Client, fossbot_name: str):
        self.response = None
        self.fossbot_name = fossbot_name
        self.event = threading.Event()
        self.socket = socket

        @self.socket.on("godotMessage")
        def godotMessage(response):
            if response["message"]["fossbot_name"] == self.fossbot_name:
                self.response = response
                self.event.set()  # signal that the event has occurred


    def post_godot(self, param: dict):
        '''
        Used to post a response from godot (POST).
        Param: param: the dictionary to be sent to godot.
        '''
        param["fossbot_name"] = self.fossbot_name
        self.socket.emit("clientMessage", param)
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
        return self.response["message"]["data"]
