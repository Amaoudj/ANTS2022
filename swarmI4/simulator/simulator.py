""" Run the simulation """
import logging
import matplotlib.pyplot as plt
from ..swarm import Swarm
# from typing import List

from ..map import Map
# from .recording import DummyRecorder


class Simulator(object):
    """ The Simulator"""

    def __init__(self, my_map: Map, display: bool):
        """ Create the simulator

        :my_map: The map generated for the simulation
        :display: If True, the simulation is displayed visually

        """
        self._my_map = my_map
        self._display = display
        self._display_initialized = False
        self._step = 0
        self._speed = 0.001

    def start(self, swarm: Swarm) -> None:
        """ Start the simulating

        :swarm: The swarm to use

        """
        self._step = 0
        logging.info("Getting the initialize position of the swarm agents")
        logging.info("Starting the simulation")

    def stop(self) -> bool:
        """ Stop the simulating (stop criteria)
        :returns: If stop criteria is reached

        """
        return self._step > 100

    def display(self):
        """ Show the world """
        if self._display:
            plt.clf()
            if not self._display_initialized:
                f = plt.gcf()
                f.set_size_inches(self._my_map.get_number_of_nodes()[0] / 4, self._my_map.get_number_of_nodes()[1] / 4,
                                  forward=True)
                f.set_dpi(100)
                self._display_initialized = True

            plt.title(f"Turn: {self._step}")
            self._my_map.view(False)
            plt.draw()
            plt.show(block=False)

    def sleep(self):
        """ Sleep for a time step """
        if self._speed > -1:
            plt.pause(self._speed)

    def main_loop(self, swarm: Swarm) -> None:
        """ The main loop of the simulator

        :swarm: The swarm

        """
        while not self.stop():
            logging.debug(f"Turn {self._step} is now running")
            self.display()
            swarm.move_all(self._my_map)
            self.sleep()
            self._step += 1

        logging.info("Simulation is done")
        if self._display:
            self.display()
            plt.show()