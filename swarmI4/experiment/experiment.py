import random
from swarmI4.agent.smart_agent import smart_agent_generator
from . base_experiment import BaseExperiment
from swarmI4.renderer.renderer_interface import RendererInterface
from swarmI4.renderer.pygame_graphics.pygame_renderer import PygameRenderer
from .. map import Map
from .. swarm import Swarm
from typing import Tuple, Callable
from .. agent import *


class Experiment(BaseExperiment):
    """
    All the basic configuration is inherited from the BaseExperiment.
    All we need to do is just to override the
    _create_swarm method to ensure that the correct agents are created.
    """

    def _create_swarm(self, args, my_map: Map):
        #print(f"Create a swarm of multiple robots\n")
        return Swarm(args, [(args.swarm_size, smart_agent_generator(args, my_map))], globals()[args.agent_placement],my_map)   # agent_placement >> custum_placement (see agent_placement in agent folde)

