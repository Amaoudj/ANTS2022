""" Main function """
import configargparse
from random import seed
import logging
from swarmI4.experiment  import *


parser = configargparse.get_arg_parser() # ArgumentParser(description="Swarm robotics for I4.0 abstract simulator.")

def parse_args():
    """ Handles the arguments """
    # parser = configargparse.get_arg_parser()


    parser.add('-c', '--config', is_config_file=True, help='Config file')

    parser.add_argument("-r", "--renderer", help="Renderer to use", nargs=1, metavar="renderer", type=str,
                        default="PygameRenderer", choices=["MatPlotLibRenderer", "NullRenderer","PygameRenderer"])


    parser.add_argument("-res", "--resolution",
                        help="width of the grid cell in px",
                        nargs=1, metavar="resolution", type=int,
                        default=20)

    parser.add_argument("-disp", "--display_size",
                        help="screen size",
                        nargs=2, metavar="display_size", type=int,
                        default=(600,1200))

    parser.add_argument("-m", "--map", help="Map/map generator to use", nargs=1, metavar="map", type=str,
                        default="WarehouseMapGenerator", choices=["WarehouseMapGenerator", "SimpleMapGenerator","CustomMapGenerator"])

    parser.add_argument("-pat", "--pattern_map_file",
                        help="Experiment to run",
                        nargs=1, metavar="pattern_map_file",
                        type=str,
                        default="map_patterns/arena.txt")
    parser.add_argument("--seed",
                        help="Random seed",
                        nargs=1, metavar="seed", type=int,
                        default=seed())

    parser.add_argument("-s", "--swarm_size",
                        help="Swarm size (number of agents)",
                        nargs=1, metavar="swarm_size", type=int,
                        default=3)

    parser.add_argument("-ta", "--num_targets",
                        help="number of targets each agent have initially",
                        nargs=1, metavar="swarm_size", type=int,
                        default=2)


    parser.add_argument("-l", "--loglevel",
                        help="Logging level",
                        nargs=1, metavar="level", choices=["INFO", "DEBUG", "WARNING", "ERROR"], type=str,
                        default="INFO")

    parser.add_argument("-p", "--agent_placement",
                        help="Agent placement function",
                        nargs=1, metavar="agent_placement", choices=["random_placement", "horizontal_placement",
                                                                     "vertical_placement",
                                                                     "center_placement",
                                                                     "custom_placement"],
                        type=str,
                        default="random_placement")

    parser.add_argument("-e", "--experiment",
                        help="Experiment to run",
                        nargs=1, metavar="experiment", choices=["BaseExperiment", "AndersTestExperiment","RaoufExperiment"],
                        type=str,
                        default="RaoufExperiment")
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

    if type(args.swarm_size) == list:
        args.swarm_size = args.swarm_size[0]

    if type(args.agent_placement) == list:
        args.agent_placement = args.agent_placement[0]

    if type(args.experiment) == list:
        args.experiment = args.experiment[0]

    if type(args.resolution) == list:
        args.resolution = args.resolution[0]

    if type(args.num_targets) == list:
        args.num_targets = args.num_targets[0]

    if type(args.pattern_map_file) == list:
        args.pattern_map_file = args.pattern_map_file[0]

    logging.basicConfig(format='%(asctime)s %(message)s')
    logging.root.setLevel(getattr(logging, args.loglevel.upper(), None))
    logging.info(f"Runtime arguments f{args}")

    my_experiment = globals()[args.experiment]()
    i=0

    while True:

        my_sim = my_experiment.create_simulator(args)
        sim_action = my_sim.start(args)

        if sim_action == 'reset':
            logging.info(f"Resetting the Simulation ...")
            continue

        sim_action = my_sim.main_loop(args)
        if sim_action == 'stop':
            break


if __name__ == "__main__":
    arg = parse_args()
    main(arg)
