""" Run the simulation """
import logging

from . renderer_interface import RendererInterface

import matplotlib.pyplot as plt

# from typing import List

from ..map import Map


class NullRenderer(RendererInterface):
    """
    A null renderer (no output)
    """

    def __init__(self, my_map, swarm):
        pass

    def setup(self,args:None):
        pass

    def tear_down(self):
        pass

    def display_frame(self, step: int):
        pass

