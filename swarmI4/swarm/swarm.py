from typing import Callable as Func, List, Tuple
from ..map import Map
from ..agent import agent_placement
from swarmI4.agent.smart_agent import SmartAgent


class Swarm(object):
    """ This a wrapper for the all agents """

    def __init__(self, agent_generators: List[Tuple[int, Func]], placement_func: Func, my_map: Map):
        """ Create the swarm """
        self._agents = []
        self._positions = {}

        self.create_swarm(agent_generators, my_map, agent_placement.random_placement)
        self._my_map = my_map

    def create_swarm(self, agent_generators: List[Tuple[int, Func]], my_map: Map, placement_func: Func) -> None:
        """ Create the swarm according to the generators """

        self._agents.clear()
        total_number_of_agents = sum([agent_type[0] for agent_type in agent_generators])

        for number, gen in agent_generators:
            for i in range(number):
                position = placement_func(i, total_number_of_agents, my_map)
                agent = gen(position)

                my_map.add_agent_to_map(agent)
                self._agents.append(agent)


    def move_all(self,dt) -> None:
        """
        Move all agents in the swarm
        :world: The world
        :returns: None
        """

        # TODO:check for conflicting neighbors of each agent
        # TODO:send data of agent to other agent using what we call msg box
        # TODO:make message box in map where each agent is able to leave messages to others
        # TODO: agents solve conflict and then move
        # TODO: Add a func to reset and run the simulation again

        # TODO: check paths when we use big AGV num

        for agent in self._agents:
            pos = agent.move(self._my_map,time_lapsed=dt)


    def set_positions(self, position: int) -> None:
        """ Set the same position for all agents in swarm
        :position: The node id
        """
        for agent in self._agents:
            agent.position = position

        self._positions.clear()
        self._positions[str(position)] = len(self._agents)

    @property
    def agents(self):
        return self._agents
