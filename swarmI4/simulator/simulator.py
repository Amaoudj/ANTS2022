""" Run the simulation """
import logging
from ..swarm import Swarm
from ..renderer import RendererInterface
# from typing import List

from ..map import Map
# from .recording import DummyRecorder
import time


class Simulator(object):
    """ The Simulator"""

    def __init__(self, my_map: Map, my_swarm: Swarm, renderer: RendererInterface):
        """ Create the simulator

        :my_map: The map generated for the simulation
        :display: If True, the simulation is displayed visually
        """
        self._my_map = my_map
        self._my_swarm = my_swarm
        self._display_initialized = False
        self._step = 0
        self._renderer = renderer

        self._start_time = 0
        self._lapsed_time = 0
        self._simulation_time = 0



    def start(self,args) -> None:
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
        return self._step > 1000

    def main_loop(self,args) -> None:

        """ The main loop of the simulator
        :swarm: The swarm
        """
        # call the setup of the _renderer
        self._renderer.setup(args)
        self._start_time = time.time()

        while not self.stop():
            time.sleep(0.1)

            logging.debug(f"Turn {self._step} is now running")
            self._renderer.display_frame(args,self._step,lapsed_time=self._lapsed_time)
            self._my_swarm.move_all(simulation_time=self._simulation_time,dt=self._lapsed_time)

            self._step += 1
            self._lapsed_time = time.time() - self._start_time
            self._simulation_time += self._lapsed_time

        self._renderer.display_frame(step=self._step,args=args)
        logging.info("Simulation is done")
        self._renderer.tear_down()
