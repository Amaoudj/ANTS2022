""" The interface for a swarm agent """
from typing import Tuple
import random

class AgentInterface:
    """ Interface class for a swarm agent """

    def __init__(self,conf: dict, position: Tuple[int, int],size:tuple=(1,1)):

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

        # added variables
        self.id = None
        self.is_selected = False
        self.wait_time = 0
        self.size = size
        self.orientation = random.randint(0,3) #[ right = 0 , up = 1 , left = 2 ,down = 3 ]

        if self._conf.get("history", False):
            self._history = []

    def move(self, world, updated_pos,time_lapsed:float=0) -> int:
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

    @property
    def row(self):
        """
        return the row of the current position
        """
        return self._position[0]

    @property
    def col(self):
        """
        return the column of the current position
        """
        return self._position[1]



    @position.setter
    def position(self, new_position: Tuple[int, int]):
        """ Set the new position

        :new_position: The new node id

        """
        if self._conf.get("history", False):
            self._history.append(new_position)
        self._position = new_position


