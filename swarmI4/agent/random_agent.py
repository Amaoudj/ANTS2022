""" A random walk agent """
import logging

from . agent_interface import AgentInterface

import random as rnd
from typing import Callable, Tuple


class RandomAgent(AgentInterface):

    """ A random walk agent """

    def move(self, my_map: map):
        """ Move the agent

        :world: The world
        :updated_pos: The updated position of agents already moved
        :returns: The new node it would move to

        """
        candidates = my_map.connected(self.position)
        new_position = rnd.choice(candidates)

        if not my_map.occupied(new_position):
            my_map.move_agent(self, new_position)

        logging.info(f"New position {new_position}")


        return self._position


def random_agent_generator() -> Callable:
    """ Create a random agent generator

    :returns: A generator function

    """

    def generator(position: Tuple[int, int]):
        return RandomAgent(None, position)

    return generator
