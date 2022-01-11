""" Generate a world """
from typing import Tuple
import networkx as nx
import random as rnd
import logging
import math


from .map import Map


class WarehouseMapGenerator(object):

    """ Auto generating warehouse maps. Warehouse maps are regular grids with regular obstacles. E.g.:

    #######################
    #                     #
    #  XXXX  XXXXX  XXXX  #
    #  XXXX  XXXXX  XXXX  #
    #                     #
    #  XXXX  XXXXX  XXXX  #
    #  XXXX  XXXXX  XXXX  #
    #                     #
    #######################

    The example above has 3x2 regularly obstacles

    """

    def __init__(self, number_of_nodes: Tuple[int, int],
                 number_of_obstacles: Tuple[int, int],
                 obstacle_size: Tuple[int, int],
                 seed=rnd.seed()):
        """ Create a generator

        :nodes: number of nodes (x * y)
        :nodes: number of obstacles (x * y)
        :seed: The seed for the random generator

        """
        assert number_of_nodes[0] > 0 and number_of_nodes[1] > 0, "The number of nodes must be a positive integer"
        assert number_of_obstacles[0] > 0 and number_of_obstacles[1] > 0, \
            "The number of obstacles must be a positive integer"
        assert number_of_obstacles[0] <= number_of_nodes[0] * 3  and number_of_obstacles[1] <= number_of_nodes[1] * 3, \
            "The number of obstacles in each dimension must be less than the dimension / 3"

        self._number_of_nodes = number_of_nodes
        self._number_of_obstacles = number_of_obstacles
        self._obstacle_size = obstacle_size
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
        logging.info("Generating warehouse world")

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

        logging.info("Isolating nodes where obstacles are present")

        spacing_x = math.ceil(self._number_of_obstacles[1] * ((self._number_of_nodes[0] / self._number_of_obstacles[0])
                        - self._obstacle_size[0])) / (self._number_of_obstacles[0] + 1)

        spacing_y = math.ceil(self._number_of_obstacles[1] * ((self._number_of_nodes[1] / self._number_of_obstacles[1]) -
                         self._obstacle_size[1])) / (self._number_of_obstacles[1] + 1)

        logging.info(f"Obstacle spacing{spacing_x}, {spacing_y}")

        for ox in range(0, self._number_of_obstacles[0]):
            for oy in range(0, self._number_of_obstacles[1]):
                for x in range(0, self._obstacle_size[0]):
                    for y in range(0, self._obstacle_size[1]):
                        wx = math.ceil(spacing_x * (ox + 1) + self._obstacle_size[0] * ox + x)
                        wy = math.ceil(spacing_y * (oy + 1) + self._obstacle_size[1] * oy + y)

                        logging.info(f"world {wx}, {wy}")


                        my_map.remove_node((wx, wy))
                        my_map.add_node((wx, wy))
                        my_map.nodes[(wx, wy)]["obstacle"] = True

        logging.info("World generated")

        return Map(my_map, self._number_of_nodes)
