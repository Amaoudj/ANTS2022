""" Run the simulation """
import logging

from . renderer_interface import RendererInterface

import matplotlib.pyplot as plt

# from typing import List

from ..map import Map


class MatplotlibRenderer (RendererInterface):
    """
    A simple matplotlib renderer
    """

    def __init__(self, my_map, swarm):
        super().__init__(my_map, swarm)
        self._display_initialized = False

    def setup(self):
        """
        Set up the rendering. Called before every new simulation.
        """

    def tear_down(self):
        """
        Tear down the rendering. Called after every simulation.
        """

    def display_frame(self, step: int):
        """
        Display a frame
        """
        plt.clf()
        if not self._display_initialized:
            f = plt.gcf()
            f.set_size_inches(self._my_map.get_number_of_nodes()[0] / 4, self._my_map.get_number_of_nodes()[1] / 4,
                              forward=True)
            f.set_dpi(100)
            self._display_initialized = True

        plt.title(f"Step: {step}")
        self._my_map.view(False)
        plt.draw()
        plt.show(block=False)
        plt.pause(0.001)


