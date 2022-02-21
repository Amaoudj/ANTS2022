""" Generate a world based on custom map"""
from typing import Tuple
import networkx as nx
import random as rnd
import logging
import math
import numpy as np
import configargparse
import os

from .map import Map

parser = configargparse.get_arg_parser()
parser.add_argument("-n", "--nodes",
                    help="The number of nodes (x, y) in the map",
                    nargs=2, metavar="nodes", type=int, default=(30, 30))

parser.add_argument("-o", "--obstacles",
                    help="The number of obstacles (x, y) in the map",
                    nargs=2, metavar="obstacles", type=int, default=(5, 5))

parser.add_argument("-z", "--obstacle_size",
                    help="Obstacle size (x, y)",
                    nargs=2, metavar="obstacle_size", type=int, default=(3, 2))


class CustomMapGenerator(object):
    """
     generating warehouse maps as a regular grids with regular obstacles:
    """

    @staticmethod
    def create_from_args(args):
        """ Static method for creating a warehouse map from args.
        """
        return CustomMapGenerator(args.pattern_map_file)

    def __init__(self, pattern_file_path:str = 'map_patterns/arena.txt'):
        """ Create a generator

        :nodes: number of nodes (x * y)
        :nodes: number of obstacles (x * y)
        :seed: The seed for the random generator
        """

        self._number_of_nodes = None
        self._number_of_obstacles = None
        self._obstacle_size = None
        self._pattern_file_path = pattern_file_path
        self.reset()

    def reset(self) -> None:
        """ Reset the generator to beginning

        """
        pass

    def read_map_pattern(self):
        """
        return:
               my_map   - matrix specifying obstacle positions
               starts   - [(x1, y1),  ...] list of start locations
               goals    - [(x1, y1),  ...] list of goal locations
        """
        if not os.path.isfile(self._pattern_file_path):
            raise Exception(f'{self._pattern_file_path} is not a file')

        f = open(self._pattern_file_path, 'r')
        # first line: #rows #columns
        line = f.readline()
        rows, columns = [int(x) for x in line.split(' ')]
        rows = int(rows)
        columns = int(columns)
        # #rows lines with the map
        my_map = []
        for r in range(rows):
            line = f.readline()
            my_map.append([])
            for cell in line:
                if cell == '@':
                    my_map[-1].append(False)
                elif cell == '.':
                    my_map[-1].append(True)
        #agents
        line = f.readline()
        num_agents = int(line)
        #agents lines with the start/goal positions
        starts = []
        goals = []
        for a in range(num_agents):
            line = f.readline()
            sx, sy, gx, gy = [int(x) for x in line.split(' ')]
            starts.append((sx, sy))
            goals.append((gx, gy))
        f.close()
        return np.array(my_map), starts, goals

    def generate(self) -> Map:
        """
        Generate a world
        """
        logging.info("Generating custom MAP")

        pattern_map,start_list,goal_list = self.read_map_pattern()
        print(start_list,goal_list)
        self._number_of_nodes = pattern_map.shape
        my_map: nx.Graph = nx.Graph()
        for x in range(0, self._number_of_nodes[0]):
            for y in range(0, self._number_of_nodes[1]):
                my_map.add_node((x, y))
                my_map.nodes[(x, y)]["agent"] = None
                #my_map.nodes[(x, y)]["obstacle"] = False
                my_map.nodes[(x, y)]["state"] = 'free_space'

                if x > 0:
                    my_map.add_edge((x - 1, y), (x, y))
                if y > 0:
                    my_map.add_edge((x, y - 1), (x, y))

        logging.info("Isolating nodes where obstacles are present")

        for row in range(0, self._number_of_nodes[0]):
            for col in range(0, self._number_of_nodes[1]):
                if not pattern_map[row,col]:
                    logging.info(f"world {row}, {col}")
                    my_map.remove_node((row, col))
                    my_map.add_node((row, col))
                    my_map.nodes[(row, col)]["obstacle"] = True
                    my_map.nodes[(row, col)]["state"] = 'obstacle'

        logging.info("World generated")
        print('start_goal_list',(start_list,goal_list))

        return Map(my_map, self._number_of_nodes,(start_list,goal_list))
