""" Run the simulation """
import logging
from ..swarm import Swarm
from ..renderer import RendererInterface
from ..renderer.pygame_graphics.utils import EventHandler
# from typing import List

from ..map import Map
# from .recording import DummyRecorder
import time
import ctypes

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
        _event_handler = EventHandler()
        sim_action = _event_handler.handle_init_events(args, self._renderer, self._my_map, self._my_swarm)
        logging.info("Getting the initialize position of the swarm agents")
        logging.info("Starting the simulation")
        return sim_action

    def stop(self) -> bool:
        """ Stop the simulating (stop criteria)
        :returns: If stop criteria is reached

        """
        return self._step > 1000

    def reset(self,args):
        """
        reset the simulation
        """
        pass

    def main_loop(self,args):

        """ The main loop of the simulator
        :swarm: The swarm
        """
        # call the setup of the _renderer
        self._renderer.setup(args)
        self._start_time = time.time()

        while not self.stop() and not self._my_swarm.done:
            time.sleep(0.5)
            logging.debug(f"Turn {self._step} is now running")
            sim_action = self._renderer.display_frame(args,self._step,lapsed_time=self._lapsed_time)
            if sim_action is not None:
                return sim_action
            self._my_swarm.move_all(simulation_time=self._simulation_time,dt=self._lapsed_time)

            self._step += 1
            self._lapsed_time = time.time() - self._start_time
            self._simulation_time += self._lapsed_time

        logging.info("Simulation is done")
        self._step -= 2  # remove the step to the start-node and the repeated step to end-node

        # The following is Windows-specific and causes a crash on Linux and MacOS:
        # ctypes.windll.user32.MessageBoxW(0, f"End Task--Time_Step : {self._step} ", "Simulation is done", 1)
        print('--------------------------------------------------')
        print('End Task--Time_Step : ', self._step)
        print('---------------------------------------------------------')
        self._renderer.display_frame(step=self._step, args=args)
        self._renderer.tear_down()
