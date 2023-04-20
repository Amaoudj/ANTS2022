""" Run the simulation """
import csv
import logging
from ..swarm import Swarm
from ..renderer import RendererInterface
from ..renderer.pygame_graphics.utils import EventHandler
from ..agent.smart_agent import SmartAgent
# from typing import List
import pandas as pd
import os,sys
from ..map import Map
# from .recording import DummyRecorder
import time
import pyautogui

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
        self.sum_cost=0
        self._renderer = renderer

        self._start_time = 0
        self._lapsed_time = 0
        self._simulation_time = 0
        self.instance=None
        self.failed=False



    def start(self,args) -> None:
        """ Start the simulating
        :swarm: The swarm to use
        """
        self._step = 0
        _event_handler = EventHandler()
        sim_action = _event_handler.handle_init_events(args, self._renderer, self._my_map, self._my_swarm)

        self.instance=args.pattern_map_file

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
        if args.renderer == "PygameRenderer":
           self._renderer.setup(args)

        self._start_time = time.time()

        self.steps_limit = 502
        sim_action=None
        while not self._my_swarm.done and self._my_swarm.success and self._step < self.steps_limit:
            #time.sleep(0.9)
            if args.renderer == "PygameRenderer":
                sim_action = self._renderer.display_frame(args,self._step,lapsed_time=self._lapsed_time)
            if sim_action is not None:
                return sim_action

            self._my_swarm.move_all(simulation_time=self._simulation_time, dt=self._lapsed_time)
            self._step += 1
            self._lapsed_time = time.time() - self._start_time
            self._simulation_time = self._lapsed_time


        if self._step >= self.steps_limit:
            logging.info(f'The number of steps has exceeded the threshold')

        if not self._my_swarm.success:
            #pyautogui.alert(text='Simulation failed', title='Simulation', button='OK')
            #logging.info(f'Simulation failed ! ')
            print(f'Simulation failed ! ')

        else:
          self._step -= 2  # remove the step to the start-node and the repeated step to end-node
          #pyautogui.alert(text='End Task--Time_Step : '+str(self._step),title='Simulation is done', button='OK')
          #logging.info(f'Simulation is done ! ')

        self.sum_cost=self._my_swarm.get_sum_cost()

        print('Simulation completed--Time-Steps : ', self._step)
        if args.renderer == "PygameRenderer":
           self._renderer.display_frame(step=self._step, args=args)
           self._renderer.tear_down()


    @property
    def is_done(self):
        """
        return the done flag
        """
        return self._my_swarm.done

    @property
    def map(self):
        return self._my_map

    @property
    def swarm(self):
        return self._my_swarm

    @property
    def step_t(self):
        return self._step

    @property
    def simulation_time(self):
        return self._simulation_time

