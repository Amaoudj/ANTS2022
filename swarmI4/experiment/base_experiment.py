from swarmI4.map import *
from swarmI4.agent import *

from swarmI4.swarm import Swarm
from swarmI4.simulator import Simulator

from swarmI4.renderer import *

from swarmI4.experiment.experiment_interface import ExperimentInterface
from swarmI4.renderer.pygame_graphics.utils import EventHandler



class BaseExperiment(ExperimentInterface):

    def __init__(self):
        pass

    def _get_map_generator(self, args):
        """
         Get the map generator for the experiment by looking up the map class specified in the args.map attribute
        """
        return globals()[args.map].create_from_args(args)

    def _create_swarm(self, args, my_map: Map):
        """ Create the swarm """
        return Swarm(args,[(args.swarm_size, random_agent_generator())], globals()[args.agent_placement], my_map)

    def _create_renderer(self, args, my_map, my_swarm) -> RendererInterface:
        """ Create the renderer """
        # I added the : _event_handler = EventHandler()
        renderer       = globals()[args.renderer](args,my_map, my_swarm)

        return renderer


    def create_simulator(self, args) -> Simulator:
        """ Create simulator  """
        my_renderer =  None
        map_generator   = self._get_map_generator(args)
        my_map: Map     = map_generator.generate()
        my_swarm: Swarm = self._create_swarm(args, my_map)

        if args.renderer == "PygameRenderer":
             my_renderer: RendererInterface = self._create_renderer(args, my_map, my_swarm)

        return Simulator(my_map, my_swarm, my_renderer)
