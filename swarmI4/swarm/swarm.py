from typing import Callable as Func, List, Tuple
from ..map import Map
from ..agent import agent_placement


class Swarm(object):
    """ This a wrapper for the all agents """

    def __init__(self, agent_generators: List[Tuple[int, Func]], placement_func: Func, my_map: Map):
        """ Create the swarm """
        self._agents = []
        self._positions = {}

        self.create_swarm(agent_generators, my_map, placement_func)
        self._my_map = my_map

    def create_swarm(self, agent_generators: List[Tuple[int, Func]], my_map: Map, placement_func: Func) -> None:
        """ Create the swarm according to the generators

        :agent_generators: a list of Tuples of [number of agents, generator function]

        """

        self._agents.clear()
        total_number_of_agents = sum([agent_type[0] for agent_type in agent_generators])

        for number, gen in agent_generators:
            for i in range(number):
                position = placement_func(i, total_number_of_agents, my_map)
                print(position)
                agent = gen(position)
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
