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


class BenchmarkMapGenerator(object):
    """
     generating warehouse maps as a regular grids with regular obstacles:
    """

    @staticmethod
    def create_from_args(args):
        """ Static method for creating a warehouse map from args.
        """
        return BenchmarkMapGenerator(args.pattern_map_file)

    def __init__(self, map:dict):
        """ Create a generator

        :nodes: number of nodes (x * y)
        :nodes: number of obstacles (x * y)
        :seed: The seed for the random generator
        """

        self._number_of_nodes = None
        self._number_of_obstacles = 0
        self._obstacle_size = None
        self._map_dict = map
        self._pattern_file_path = self._map_dict['map_file']
        self.number_of_targets = None

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
        rows, columns = [line.split(' ')[0],line.split(' ')[1]]
        rows = int(rows)
        columns = int(columns)
        # #rows lines with the map
        my_map = []
        for r in range(rows):
            line = f.readline()
            my_map.append([])
            for cell in line:
                if cell == '@' or cell == 'T':
                    my_map[-1].append(False)
                elif cell == '.':
                    my_map[-1].append(True)
        #agents lines with the start/goal positions

        starts = []
        goals = []
        for pos in self._map_dict['poses']:
            starts.append((pos[1],pos[0]))
            goals.append((pos[3],pos[2]))
        f.close()

        return np.array(my_map), starts, goals, self._pattern_file_path

    def generate(self) -> Map:
        """
        Generate a world
        """

        pattern_map,start_list,goal_list,pattern_file_path = self.read_map_pattern()
        self.number_of_targets = len(goal_list)//len(start_list)
        self._number_of_nodes = pattern_map.shape

        logging.info(self._number_of_nodes)

        my_map: nx.Graph = nx.Graph()
        copy_graph: nx.Graph = nx.Graph()
        for x in range(0, self._number_of_nodes[0]):
            for y in range(0, self._number_of_nodes[1]):
                my_map.add_node((x, y))
                my_map.nodes[(x, y)]["agent"] = None
                my_map.nodes[(x, y)]["obstacle"] = False
                my_map.nodes[(x, y)]["state"] = 'free_space'

                copy_graph.add_node((x, y))
                copy_graph.nodes[(x, y)]["obstacle"] = False
                copy_graph.nodes[(x, y)]["agent"] = None
                copy_graph.nodes[(x, y)]["state"] = 'free_space'

                if x > 0:
                    my_map.add_edge((x - 1, y), (x, y), weight=1)
                    copy_graph.add_edge((x - 1, y), (x, y), weight=1)
                if y > 0:
                    my_map.add_edge((x, y - 1), (x, y), weight=1)
                    copy_graph.add_edge((x, y - 1), (x, y), weight=1)

        #logging.info("Isolating nodes where obstacles are present")

        for row in range(0, self._number_of_nodes[0]):
            for col in range(0, self._number_of_nodes[1]):
                if not pattern_map[row,col]:
                    # logging.info(f"world {row}, {col}")
                    my_map.remove_node((row, col))
                    #######################################################################
                    copy_graph.remove_node((row, col))
                    ########################################################################
                    my_map.add_node((row, col))
                    my_map.nodes[(row, col)]["obstacle"] = True
                    my_map.nodes[(row, col)]["agent"] = None
                    my_map.nodes[(row, col)]["state"] = 'obstacle'
                    self._number_of_obstacles+=1

        logging.info("World generated")

        return Map(my_map,copy_graph, self._number_of_nodes,(start_list,goal_list),self._number_of_obstacles,self.number_of_targets,pattern_file_path)

