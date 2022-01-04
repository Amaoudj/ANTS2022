""" Contains the world description for the swarm """
from typing import Tuple
import networkx as nx
# from networkx.drawing.nx_agraph import graphviz_layout
import logging
from .. agent import AgentInterface

from .color import FREE, AGENT


class Map(object):

    """ The world representation """

    def __init__(self, graph, number_of_nodes: Tuple[int, int]):
        """TODO: to be defined. """
        logging.debug("Initializing the world")
        self._graph = graph
        self._number_of_nodes = number_of_nodes

    def get_number_of_nodes(self) -> Tuple[int, int]:
        return self._number_of_nodes

    def connected(self, node: int) -> list:
        """ Get the corrected nodes to node

        :node: The node id
        :returns: list of node

        """
        return list(nx.neighbors(self._graph, node))

    def occupied(self, position: Tuple[int, int]):
        """
        Check if a node is occupied
        :position: node position
        """
        return self._graph.nodes[position]["agent"] is not None

    def move_agent(self, agent: AgentInterface, new_position: Tuple[int, int]):
        assert self._graph.nodes[agent.position]["agent"] == agent, \
            f"Error, agent is not currently located at {agent.position}"
        assert self._graph.nodes[new_position]["agent"] is None, \
            f"Error, location is not free {new_position}"

        self._graph.nodes[agent.position]["agent"] = None
        self._graph.nodes[new_position]["agent"] = agent
        agent.position = new_position

    def add_agent_to_map(self, agent: AgentInterface):
        assert self._graph.nodes[agent.position]["agent"] is None, \
            f"Trying to place an agent at a none free location {agent.position}"
        self._graph.nodes[agent.position]["agent"] = agent


    def size(self) -> int:
        """ Get the size of the world """
        return nx.number_of_nodes(self._graph)

    def cost(self, node_1, node_2) -> int:
        """ Get the cost between two nodes

        :node_1: Node id
        :node_2: NOde id
        :returns: The cost

        """
        return self._graph[node_1][node_2]["weight"]

    def get_agents_numbers(self, node):
        """ Get the number of agents and a given node

        :node: The node id
        :returns: The number of agents

        """
        assert 0 <= node <= nx.number_of_nodes(self._graph), "Get number of agents from a non existing node"
        ret = self._graph.nodes[node].get("agents")
        if ret is None:
            return 0

        return ret

    def update_value(self, node: int, key: str, value):
        """ Update a value on a node

        :node: The node id
        :key: The key
        :value: The value to assign

        """
        node = int(node)
        assert 0 <= node <= nx.number_of_nodes(self._graph), "Updating value on a non existing node"
        self._graph.nodes[node][key] = value

    def view(self, block=True):
        """ Show the world

        """
        color = self._graph.nodes(data="agent", default=None)
        color = [FREE if c is None else AGENT for _, c in color]

        pos = {n: (n[0] * 10, n[1] * 10) for n in nx.nodes(self._graph)}

        nodes_graph = nx.draw_networkx_nodes(self._graph, pos=pos, node_color=color,
                                             node_size=300, node_shape="s", linewidths=1.0)

        nodes_graph.set_edgecolor('black')
