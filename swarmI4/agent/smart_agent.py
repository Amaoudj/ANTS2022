""" A smart Agent"""
import logging

from . agent_interface import AgentInterface

from swarmI4.calculation.path_planning import PathFinder

from typing import Callable, Tuple


class SmartAgent(AgentInterface):

    def __init__(self,conf,my_map,position,num_targets):
        super().__init__(conf,position)

        self._path_finder = PathFinder()
        # initialize targets
        self.num_targets = num_targets
        self.target_list = []

        for n in range(0, self.num_targets):
            target = my_map.get_free_node()
            self.target_list.append(target)

        # initial path info
        self.path = None
        self._current_target_id = 0
        self._current_waypoint_id = 0

    def move(self,map, time_lapsed:float=0):

        """ Move the agent
        :env: The env
        :updated_pos: The updated position of agents already moved
        :returns: The new node it would move to
        """

        if self.path is None:
             self.plan(map)
        target   = self.target_list[self._current_target_id]
        waypoint = self.path[self._current_waypoint_id]

        # required for MatPlotLibRender
        map._graph.nodes[(self.row, self.col)]["agent"] = None
        map._graph.nodes[waypoint]["agent"] = self

        self.face_to(waypoint)
        self.step(map,target_pos=target,waypoint=waypoint)


    def plan(self,map)->list:
        """
        method for planning the agent's path to its targets
        :param path_finder: the astar planner
        :return:
        """
        # plan a path to the next target
        target = self.target_list[self._current_target_id]
        self.path = self._path_finder.astar_planner(map.graph, self.position, target)
        map.set_as_path(self.path)

        return self.path


    def wait(self,wait_time):
        # used later
        self.wait_time += wait_time

    def step(self,map, target_pos=None, waypoint=None):

        self._position = waypoint # update the agent pos required to display the agent

        if (waypoint == target_pos):

            # set the old path nodes as free again
            if self.path is not None:
                old_path = self.path
                map.set_as_free(old_path)
                map.set_as_free(target_pos)
            if self._current_target_id + 1 < len(self.target_list):
                logging.info(f'heading towards target: {self._current_target_id + 1}')
                self._current_target_id += 1
                self._current_waypoint_id = 0
                self.plan(map)
                return

        if self._current_waypoint_id + 1 < len(self.path):
            self._current_waypoint_id += 1
            return

        return self._position


    def face_to(self,position):
        row,col = position
        if row < self.row and col == self.col:
            self.orientation = 1        # face up
        elif row > self.row and col == self.col:
            self.orientation = 3        # face down
        elif row == self.row and col > self.col:
            self.orientation = 0        # face right
        elif row == self.row and col < self.col:
            self.orientation = 2        # face left


    @property
    def target_id(self):
        """
        return the current target id
        :return:
        """
        return self._current_target_id


def smart_agent_generator(args,my_map) -> Callable:
    """Create a random agent generator
    :returns: A generator function
    """
    def generator(position: Tuple[int, int]):
        return SmartAgent(None,my_map, position=position, num_targets=args.num_targets)

    return generator


