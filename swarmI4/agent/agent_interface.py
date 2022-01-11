""" The interface for a swarm agent """
from typing import Tuple


class AgentInterface:
    """ Interface class for a swarm agent """

    def __init__(self, conf: dict, position: Tuple[int, int]):
        """ Create the agent

        :conf: The configuration of agent
        :position: The starting position of the agent

        """
        if conf is not None:
            assert type(conf) is dict, "The agent configuration must be of the type dict"

        if conf is None:
            conf = {}

        self._conf = conf
        self._position = position

        if self._conf.get("history", False):
            self._history = []

    def move(self, world, updated_pos) -> int:
        """ Move the agent

        :world: The world
        :updated_pos: The updated position of agents already moved
        :returns: The new node it would move to

        """
        return NotImplementedError("Swarm agents should contains this method")

    @property
    def position(self) -> Tuple[int, int]:
        """ The position of the agent """
        return self._position

    @position.setter
    def position(self, new_position: Tuple[int, int]):
        """ Set the new position

        :new_position: The new node id

        """
        if self._conf.get("history", False):
            self._history.append(new_position)
        self._position = new_position
