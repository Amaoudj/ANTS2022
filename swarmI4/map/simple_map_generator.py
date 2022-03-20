""" Generate a world """
from typing import Tuple
import networkx as nx
import random as rnd
import logging
import configargparse

from .map import Map

parser = configargparse.get_arg_parser()
parser.add_argument("-n", "--nodes",
                    help="The number of nodes (x, y) in the map",
                    nargs=2, metavar="nodes", type=int, default=(10, 10))


class SimpleMapGenerator(object):

    @staticmethod
    def create_from_args(args):
        """ Static method for creating a simple map from args.
        """
        return SimpleMapGenerator(args.nodes, args.seed)

    """ Auto generate simple maps (no obstacles or similar) """
    def __init__(self, number_of_nodes: Tuple[int, int],
                 seed=rnd.seed()):
        """ Create a generator

        :nodes: number of nodes (x * y)
        :seed: The seed for the random generator

        """
        assert number_of_nodes[0] > 0 and number_of_nodes[1] > 0, "The number of nodes must be a positive integer"

        self._number_of_nodes = number_of_nodes
        self._seed = seed
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
        copy_graph: nx.Graph = nx.Graph()
        for x in range(0, self._number_of_nodes[0]):
            for y in range(0, self._number_of_nodes[1]):
                my_map.add_node((x, y))

                my_map.nodes[(x, y)]["obstacle"] = False
                my_map.nodes[(x, y)]["agent"] = None
                my_map.nodes[(x, y)]["state"] = 'free_space'

                copy_graph.add_node((x, y))
                copy_graph.nodes[(x, y)]["obstacle"] = False
                copy_graph.nodes[(x, y)]["agent"] = None
                copy_graph.nodes[(x, y)]["state"] = 'free_space'
#                logging.info(f"Adding {x} and {y}")
                if x > 0:
                    my_map.add_edge((x - 1, y), (x, y), weight=1)
                    copy_graph.add_edge((x - 1, y), (x, y), weight=1)
                if y > 0:
                    my_map.add_edge((x, y - 1), (x, y), weight=1)
                    copy_graph.add_edge((x, y - 1), (x, y), weight=1)

        logging.info("World generated")

        return Map(my_map, self._number_of_nodes,number_of_obstacles=0)
