""" Run the simulation """
import logging
from ..swarm import Swarm
from ..renderer import RendererInterface

from ..map import Map
# from .recording import DummyRecorder


class Simulator(object):
    """ The Simulator"""

    def __init__(self, my_map: Map, renderer: RendererInterface):
        """ Create the simulator

        :my_map: The map generated for the simulation
        :display: If True, the simulation is displayed visually

        """
        self._my_map = my_map
        self._display_initialized = False
        self._step = 0
        self._renderer = renderer

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

    def main_loop(self, swarm: Swarm) -> None:
        """ The main loop of the simulator

        :swarm: The swarm

        """
        self._renderer.setup()

        while not self.stop():
            logging.debug(f"Turn {self._step} is now running")
            self._renderer.display_frame(self._step)
            swarm.move_all()
            self._step += 1

        self._renderer.display_frame(self._step)
        logging.info("Simulation is done")
        self._renderer.tear_down()
