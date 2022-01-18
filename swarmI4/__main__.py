""" Main function """
import configargparse
from random import seed
import logging

from .map import *

from . agent import random_agent_generator
from . swarm import Swarm
from . simulator import Simulator

from . renderer import *

# from . import VideoRecorder, DummyRecorder

parser = configargparse.get_arg_parser() # ArgumentParser(description="Swarm robotics for I4.0 abstract simulator.")


def parse_args():
    """ Handles the arguments """
    parser = configargparse.get_arg_parser() # configargparse.ArgumentParser(description="Swarm robotics for I4.0 abstract simulator.")

    parser.add('-c', '--config', is_config_file=True, help='Config file')

    parser.add_argument("-r", "--renderer", help="Renderer to use", nargs=1, metavar="renderer", type=str,
                        default="MatPlotLibRenderer", choices=["MatPlotLibRenderer", "NullRenderer"])

    parser.add_argument("-m", "--map", help="Map/map generator to use", nargs=1, metavar="map", type=str,
                        default="WarehouseMapGenerator", choices=["WarehouseMapGenerator", "SimpleMapGenerator"])

    parser.add_argument("--seed",
                        help="Random seed",
                        nargs=1, metavar="seed", type=int,
                        default=seed())

    parser.add_argument("-l", "--loglevel",
                        help="Logging level",
                        nargs=1, metavar="level", choices=["INFO", "DEBUG", "WARNING", "ERROR"], type=str,
                        default="INFO")

    # subparse_run.add_argument("-s", "--swarm",
    #                           help="The size of the swarm",
    #                           nargs=1, metavar="size", type=int, required=True)
    #

    return parser.parse_args()


def main(args):
    if type(args.loglevel) == list:
        args.loglevel = args.loglevel[0]

    if type(args.map) == list:
        args.map = args.map[0]

    if type(args.renderer) == list:
        args.renderer = args.renderer[0]

    if type(args.seed) == list:
        args.seed = args.seed[0]

    logging.basicConfig(format='%(asctime)s %(message)s')
    logging.root.setLevel(getattr(logging, args.loglevel.upper(), None))
    logging.info(f"Runtime arguments f{args}")

    map_generator = globals()[args.map].create_from_args(args)

    my_map: Map = map_generator.generate()
    my_swarm: Swarm = Swarm([[10, random_agent_generator()]], my_map, )

    my_renderer: RendererInterface = globals()[args.renderer](my_map, my_swarm)

    my_sim: Simulator = Simulator(my_map, my_renderer)
    my_sim.start(my_swarm)
    my_sim.main_loop(my_swarm)


if __name__ == "__main__":
    arg = parse_args()
    main(arg)
