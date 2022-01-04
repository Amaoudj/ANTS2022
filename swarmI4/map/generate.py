""" Generate a world """
from typing import Tuple
import networkx as nx
import random as rnd
import logging

from .map import Map


class MapGenerator(object):

    """ Auto generating worlds"""

    def __init__(self, number_of_nodes: Tuple[int, int],
                 edge_multiplier: float, weight_range: Tuple[int, int],
                 seed=rnd.seed()):
        """ Create a generator

        :nodes: number of nodes (x * y)
        :edge_multiplier: The maximum number of edges is calculated by nodes * edge_multiplier
        :weight_range: The range of the weight for the edges
        :seed: The seed for the random generator

        """
        assert number_of_nodes[0] > 0 and number_of_nodes[1] > 0, "The mini number of nodes must be a positive integer"
        assert edge_multiplier > 0, "The edge multiplier must be a positive number"
        assert type(weight_range) == tuple and len(weight_range) == 2,  \
            "The weight range is either not a tuple of doesn't have the correct size"
        assert weight_range[0] <= weight_range[1], "The first integer in the range must be the smallest"

        self._number_of_nodes = number_of_nodes
        self._edge_multiplier = edge_multiplier
        self._seed = seed
        self._min_weight, self._max_weight = weight_range
        self.reset()

    def reset(self) -> None:
        """ Reset the generator to beginning

        """
        rnd.seed(self._seed)

    def generate(self) -> Map:
        """ Generate a world
        :returns: TODO

        """
        logging.info("Generating world")

        edge_count = int(1 + (3 * self._number_of_nodes[0] * self._number_of_nodes[1] + 1) / 2)

        logging.info(f'''Map generator settings: Seed: {self._seed} Number nodes: {self._number_of_nodes}'''
                     f''' Edge count: {edge_count} ''')

        logging.info("Generating regular lattice")
        my_map: nx.Graph = nx.Graph()
        for x in range(0, self._number_of_nodes[0]):
            for y in range(0, self._number_of_nodes[1]):
                my_map.add_node((x, y))
                my_map.nodes[(x, y)]["agent"] = None
#                logging.info(f"Adding {x} and {y}")
                if x > 0:
                    my_map.add_edge((x - 1, y), (x, y))
                if y > 0:
                    my_map.add_edge((x, y - 1), (x, y))

        logging.info("World generated")

        return Map(my_map, self._number_of_nodes)
