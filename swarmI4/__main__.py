""" Main function """
import argparse
from random import seed
import logging

from .map import Map, MapGenerator
import matplotlib.pyplot as plt

from . agent import random_agent_generator
from . swarm import Swarm
from . simulator import Simulator

# from . import VideoRecorder, DummyRecorder


def parse_args():
    """ Handles the arguments """
    parser = argparse.ArgumentParser(description="Swarm robotics for I4.0 abstract simulator.")

    subparsers = parser.add_subparsers(help="Commands")
    subparse_run = subparsers.add_parser("run", help="Run a experiment")

    subparse_run.add_argument("-n", "--nodes",
                              help="The number of nodes (x, y) in the map",
                              nargs=2, metavar="nodes", type=int, default=(10, 10))

    subparse_run.add_argument("--no-display",
                              help="Run in headless mode without display",
                              action="store_const",
                              const=True)

    subparse_run.add_argument("--seed",
                              help="Random seed",
                              nargs=1, metavar="seed", type=int,
                              default=seed())
    # subparse_run.add_argument("--delay",
    #                           help="The delay between each turn in the simulation (used together with display)",
    #                           nargs=1, metavar="delay", type=float, default=-1)
    # subparse_run.add_argument("-r", "--record",
    #                           help="Record a video of the simulation",
    #                           nargs=1, metavar="filename", type=str)
    #
    # subparse_run.add_argument("-s", "--swarm",
    #                           help="The size of the swarm",
    #                           nargs=1, metavar="size", type=int, required=True)
    #
    subparse_run.add_argument("-l", "--loglevel",
                              help="Logging level",
                              nargs=1, metavar="level", choices=["INFO", "DEBUG", "WARNING", "ERROR"], type=str,
                              default="INFO")

    subparse_run.set_defaults(func=main)

    return parser.parse_args()


def main(args):
    if type(args.loglevel) == list:
        args.loglevel = args.loglevel[0]

    if type(args.seed) == list:
        args.seed = args.seed[0]

    logging.basicConfig(format='%(asctime)s %(message)s')
    logging.root.setLevel(getattr(logging, args.loglevel.upper(), None))
    logging.info(f"Runtime arguments f{args}")

    map_generator = MapGenerator(args.nodes, 2,
                                 (2, 3), args.seed)

    my_map: Map = map_generator.generate()
    my_swarm: Swarm = Swarm([[1, random_agent_generator()]], my_map)
    my_sim: Simulator = Simulator(my_map, True)
    my_sim.start(my_swarm)
    my_sim.main_loop(my_swarm)


"""    
    plt.title("Testing")
    my_map.view()
    plt.draw()
    plt.savefig("test.png")
    plt.show()
"""

if __name__ == "__main__":
    arg = parse_args()
    arg.func(arg)
