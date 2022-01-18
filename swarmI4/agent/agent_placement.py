import random

from .. agent import AgentInterface
from typing import Tuple
from random import seed


def random_placement(agent_number: int, my_map) -> Tuple[int, int]:
    tries = 0
    placed = False

    while not placed and tries < 1000:
        x, y = random.randint(0, my_map.size_x - 1), random.randint(0, my_map.size_y - 1)
        if not my_map.occupied((x, y)):
            placed = True
        else:
            tries += 1

    if not placed:
        assert False, f"Tried to place agent number {agent_number} {tries} times, but without luck"

    return x, y


def horizontal_placement(agent_number: int, my_map) -> Tuple[int, int]:
    return agent_number, 0



