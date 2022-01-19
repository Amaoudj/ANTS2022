""" Experiments create everything  where everything is setup according to commandline args. Override this class
(or copy and modify) to create your own specialized experiment with a unique setup """

from .. map import *
from .. agent import *

from .. swarm import Swarm
from .. simulator import Simulator

from .. renderer import *


class ExperimentInterface:
    """ Interface class for a swarm agent """
    def __init__(self):
        pass

    def _get_map_generator(self, args):
        assert False, "Experiment should override this method"

    def _create_swarm(self, args, my_map: Map):
        assert False, "Experiment should override this method"

    def _create_renderer(self, args, my_map, my_swarm) -> RendererInterface:
        assert False, "Experiment should override this method"

    def create_simulator(self, args) -> Simulator:
        assert False, "Experiment should override this method"
