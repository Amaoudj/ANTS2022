from typing import Callable as Func, List, Tuple
from ..map import Map
from ..agent import agent_placement


class Swarm(object):
    """ This a wrapper for the all agents """

    def __init__(self, agent_generators: List[Tuple[int, Func]], my_map: Map):
        """ Create the swarm """
        self._agents = []
        self._positions = {}

        self.create_swarm(agent_generators, my_map, agent_placement.random_placement)
        self._my_map = my_map

    def create_swarm(self, agent_generators: List[Tuple[int, Func]], my_map: Map, placement_func: Func) -> None:
        """ Create the swarm according to the generators """

        self._agents.clear()

        for number, gen in agent_generators:
            for i in range(number):
                agent = gen(agent_placement.horizontal_placement(i, my_map))
                my_map.add_agent_to_map(agent)
                self._agents.append(agent)

    def move_all(self) -> None:
        """ Move all agents in the swarm

        :world: The world
        :returns: None

        """
        for agent in self._agents:
            pos = agent.move(self._my_map)

    def set_positions(self, position: int) -> None:
        """ Set the same position for all agents in swarm

        :position: The node id

        """
        for agent in self._agents:
            agent.position = position

        self._positions.clear()
        self._positions[str(position)] = len(self._agents)
