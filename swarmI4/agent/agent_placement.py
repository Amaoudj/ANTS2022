import math
import random
from typing import Tuple

MAX_PLACEMENT_TRIES = 1000

"""
Initial agent placement functions. When agents are created and initially placed, a placement function is called
iteratively for each agent to determine its initial coordinates.
"""


def random_placement(agent_number: int, total_number_of_agents: int, my_map) -> Tuple:
    """ Randomly place agents across the whole maps. x and y are sampled from a uniform distribution until a free
    spot is found. If a free spot has not been found after MAX_PLACEMENT_TRIES tries, we give up and an assertion
    fails.
    :agent_number: the number of the agent currently being placed (0...total number of agents - 1)
    :total_number_of_agents: the total number of agents that will be placed
    :my_map: the map
    """

    tries = 0
    placed = False

    while not placed and tries < MAX_PLACEMENT_TRIES:
        x, y = random.randint(0, my_map.size_x - 1), random.randint(0, my_map.size_y - 1)
        if not my_map.occupied((x, y)):
            placed = True
        else:
            tries += 1

    if not placed:
        assert False, f"Tried to place agent number {agent_number} {tries} times, but without luck"

    return (x, y),None


def horizontal_placement(agent_number: int, total_number_of_agents: int, my_map) -> Tuple:
    """ Place agents horizontally centered in row 0 in the environment

    :agent_number: the number of the agent currently being placed (0...total number of agents - 1)
    :total_number_of_agents: the total number of agents that will be placed
    :my_map: the map
    """

    if agent_number % 2 == 0:
        return (int(my_map.size_x / 2 + agent_number / 2 + 1), 0),None
    else:
        return (int(my_map.size_x / 2 - agent_number / 2), 0),None


def vertical_placement(agent_number: int, total_number_of_agents: int, my_map) -> Tuple:
    """ Place agents vertically centered in column 0 in the environment

    :agent_number: the number of the agent currently being placed (0...total number of agents - 1)
    :total_number_of_agents: the total number of agents that will be placed
    :my_map: the map
    """

    if agent_number % 2 == 0:
        return (0, int(my_map.size_y / 2 + agent_number / 2 + 1)),None
    else:
        return (0, int(my_map.size_y / 2 - agent_number / 2)),None


def center_placement(agent_number: int, total_number_of_agents: int, my_map) -> Tuple:
    """ Place agents vertically centered in column 0 in the environment

    :agent_number: the number of the agent currently being placed (0...total number of agents - 1)
    :total_number_of_agents: the total number of agents that will be placed
    :my_map: the map
    """

    square_side_length = int(math.sqrt(total_number_of_agents))
    column = agent_number % square_side_length
    row = int(agent_number / square_side_length)
    x = int(my_map.size_x / 2 - square_side_length / 2) + column
    y = int(my_map.size_y / 2 - square_side_length / 2) + row
    return (x, y),None

def custom_placement(agent_number: int, total_number_of_agents: int, my_map) -> Tuple:
    """ Place agents and their targets according to a custom map
    :my_map: the map
    """
    start    = my_map.custom_start_list.pop(0)
    targets  = my_map.custom_goal_list.pop(0)

    return start,targets




