"""
A simple test of how experiment configurations and agents can be customized. In this experiment, a custom agent type is
created.
"""

import random

from . base_experiment import BaseExperiment
from .. map import Map
from .. swarm import Swarm
from typing import Tuple, Callable
from .. agent import *


class AndersTestAgent(AgentInterface):
    """
    Custom agent type that moves either up, down, left, right or random until an obstacle is hit.
    """

    def __init__(self, my_map = Map):
        super(AndersTestAgent, self).__init__(None, my_map)
        self._direction = self._new_direction()


    def _new_direction(self) -> str:
        """
        Pick a new direction to move in.
        """
        return random.choice(["UP", "DOWN", "LEFT", "RIGHT", "RANDOM"])

    def move(self, my_map: map):
        x, y = self._position
        if self._direction == "UP":
            y += 1
        if self._direction == "DOWN":
            y -= 1
        if self._direction == "LEFT":
            x -= 1
        if self._direction == "RIGHT":
            x += 1
        if self._direction == "RANDOM":
            x += random.choice([-1, 1])
            y += random.choice([-1, 1])

        if not my_map.occupied((x, y)):
            my_map.move_agent(self, (x, y))
        else:
            self._direction = self._new_direction()


def anders_test_agent_generator() -> Callable:
    """
    Factory function to create agents if the right type

    @return: a generator for AndersTestAgent
    """

    def generator(position: Tuple[int, int]):
        return AndersTestAgent(position)

    return generator


class AndersTestExperiment(BaseExperiment):
    """
    All the basic configuration is inherited from the BaseExperiment. All we need to do is just to override the
    _create_swarm method to ensure that the correct agents are created.
    """

    def _create_swarm(self, args, my_map: Map):
        return Swarm([[args.swarm_size, anders_test_agent_generator()]], globals()[args.agent_placement], my_map)




