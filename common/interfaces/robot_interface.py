"""
Interface for FossBot
"""

from abc import ABC, abstractmethod

class FossBotInterface(ABC):
    """ FossBot Interface """

    @abstractmethod
    def get_distance(self) -> float:
        """ Return Distance in cm  """

    @abstractmethod
    def check_for_obstacle(self) -> bool:
        """ Return if obstacle detected """

    @abstractmethod
    def just_move(self, direction: str = "forward") -> None:
        """ Move forward forever """

    @abstractmethod
    def stop(self) -> None:
        """ Stop moving """
