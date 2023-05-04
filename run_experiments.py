""" Main function """
import os
import sys
import re
#sys.path.append('../swarm4I40sim')
#sys.path.append('../swarm4I40sim/swarmI4')
import pandas as pd
import configargparse
from random import seed
import logging
import multiprocessing

from swarmI4.statistics.stat_plotter import StatisticsPlotter
import swarmI4
from swarmI4.agent.smart_agent import SmartAgent
import time
import PySimpleGUI as sg
from swarmI4.experiment  import *
import numpy as np
import json
import csv

parser = configargparse.get_arg_parser()
RESULTS_PATH = 'results_plot/results_data_solvers/DCMAPF/results.csv'
MAP_STORAGE_PATH = 'conf_experiments/maps_storage'
BENCHMARK_STORAGE_PATH = 'benchmarks'
PLOTS_STORAGE_PATH = 'conf_experiments/plots_storage'


def export_results(args, map, swarm, step, simulation_time, storage_path: str = 'results_plot/results_data_solvers/DCMAPF/results.csv'):
    """
    store the simulation results
    """
    file_path=str(map.pattern_file_path).replace("txt", "map")
    file_path = file_path.replace("benchmarks\\", "")
    data = {'map_name': file_path.replace("benchmarks/", ""),
            'map_size': map.size_x*map.size_y,
            'obstacles_number': map.number_of_obstacles,
            'num_agents': len(swarm.agents),
            'solver':"DCMAPF",
            'solved': swarm.done,
            'soc': swarm.get_sum_cost(),
            'makespan': step,
            'simulation_time': round(simulation_time,1)}
    data = {k: [v] for k, v in data.items()}
    # 'num_replanned_paths':swarm.get_num_replanned_paths(),
    df = pd.DataFrame(data)


    if os.path.isfile(storage_path):
        df.to_csv(storage_path, mode='a', index=False, header=False)
    else:
        df.to_csv(storage_path, mode='w', index=False)

def store_map_txt(map_storage_directory,map_rep):
    """
    Store the created map representation
    into a txt file for later use
    """
    file_id = 1
    file_path = os.path.join(map_storage_directory, f'map_{file_id}.txt')
    if os.path.isdir(map_storage_directory):
        while os.path.isfile(file_path):
            file_id += 1
            file_path = os.path.join(map_storage_directory, f'map_{file_id}.txt')
    else:
        os.mkdir(map_storage_directory)
    # writing the information into the txt file
    with open(file_path, 'w') as f:
        for row in map_rep:
            for element in row:
                f.write(element)
                f.write(' ')
            f.write('\n')


def save_map_pattern(map, swarm):
    """
    save this map and swarm info into a file
    '@' = obstacle,   '.' = free_space
    """
    nodes = map.nodes
    new_map = []
    new_map.append([str(map.size_x), str(map.size_y)])
    # representing nodes states by symbols
    for row in range(0,map.size_x):
        map_line = []

        for col in range(0, map.size_y):
            if nodes[(row,col)]['state'] == 'free_space':
                map_line.append('.')
            elif nodes[(row,col)]['state'] == 'obstacle':
                map_line.append('@')
        new_map.append(map_line)

    new_map.append([str(len(swarm.agents))])
    for agent in swarm.agents:
        agent_info = []
        agent_info = agent_info + [str(agent.position[0]),str(agent.position[1])]


        if type(agent) is swarmI4.agent.smart_agent.SmartAgent:
            for target in agent.target_list:
                agent_info = agent_info + [str(target[0]),str(target[1])]
        new_map.append(agent_info)
    return new_map



def parse_args():
    """ Handles the arguments """
    # parser = configargparse.get_arg_parser()

    parser.add('-c', '--config', is_config_file=True, help='Config file')

    parser.add_argument("-r", "--renderer", help="Renderer to use", nargs=1, metavar="renderer", type=str,
                        default="PygameRenderer", choices=["MatPlotLibRenderer", "NullRenderer","PygameRenderer"])

    parser.add_argument("-plt_x", "--plot_x_axis", help="variables to be assigned to the x axis of the plot",
                        nargs='+', metavar="plot_x_axis", type=str)

    parser.add_argument("-plt_y", "--plot_y_axis", help="variables to be assigned to the y axis of the plot",
                        nargs='+', metavar="plot_y_axis", type=str)

    parser.add_argument("-plt_cat", "--plot_categories", help="plot categories",
                        nargs='+', metavar="plot_categories", type=str)



    parser.add_argument("-res", "--resolution",
                        help="width of the grid cell in px",
                        nargs=1, metavar="resolution", type=int,
                        default=20)

    parser.add_argument("-disp", "--display_size",
                        help="screen size",
                        nargs=2, metavar="display_size", type=int,
                        default=(600,1200))

    parser.add_argument("-m", "--map", help="Map/map generator to use", nargs=1, metavar="map", type=str,
                        default="WarehouseMapGenerator", choices=["WarehouseMapGenerator", "SimpleMapGenerator","CustomMapGenerator","BenchmarkMapGenerator"])



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

    parser.add_argument("-nrm", "--num_random_maps",
                        help="number of map to create randomly without modification from the user",
                        nargs=1, metavar="num_random_maps", type=int,
                        default=3)

    parser.add_argument("-ta", "--num_targets",
                        help="number of targets each agent have initially",
                        nargs=1, metavar="swarm_size", type=int,
                        default=2)

    parser.add_argument("-batch", "--batch_size",
                        help="number of simulations executed at once",
                        nargs=1, metavar="batch_size", type=int,
                        default=5)

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

    parser.add_argument("-cmap", "--create_map",
                        help="if the simulation is successful store it's map",
                        nargs=1, metavar="create_map", type=int,
                        default=0)

    parser.add_argument("-runex", "--run_experiments",
                        help="run multiple simulations and store it's data",
                        nargs=1, metavar="run_experiments", type=int,
                        default=0)

    parser.add_argument("-pe", "--plot_experiments",
                        help="plot experiments on/off",
                        nargs=1, metavar="plot_experiments", type=int,
                        default=0)


    parser.add_argument("-maps", "--maps_to_run",
                        help="list of maps to use when running multiple successive simulations",
                        nargs='+', metavar="maps_to_run",
                        type=list)

    parser.add_argument("-robset", "--robot_set",
                        help="numbers of robots to spawn eatch time",
                        nargs='+', metavar="robot_set",
                        type=json.loads)

    return parser.parse_args()


def main(args,id=None,map = None, is_benchmark:bool=True):
    if type(args.loglevel) == list:
        args.loglevel = args.loglevel[0]

    if type(args.map) == list:
        args.map = args.map[0]

    if type(args.renderer) == list:
        args.renderer = args.renderer[0]

    if type(args.plot_x_axis) == list:
        args.plot_x_axis = args.plot_x_axis[0]

    if type(args.plot_y_axis) == list:
        args.plot_y_axis = args.plot_y_axis[0]

    if type(args.plot_categories) == list:
        args.plot_categories = args.plot_categories[0]

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

    if type(args.create_map) == list:
        args.create_map = args.create_map[0]

    if type(args.batch_size) == list:
        args.batch_size = args.batch_size[0]

    if type(args.run_experiments) == list:
        args.run_experiments = args.run_experiments[0]

    if type(args.plot_experiments) == list:
        args.plot_experiments = args.plot_experiments[0]

    if type(args.num_random_maps) == list:
        args.num_random_maps = args.num_random_maps[0]

    if type(args.maps_to_run) == list:
        args.maps_to_run = args.maps_to_run[0]


    if map != None:

        args.pattern_map_file = map  # it iss the scenario to run, which is returned by the function get_benchmarks_data


    parser.add_argument("-pid", "--process_id",
                        help="the id of the process running ",
                        nargs=1, metavar="process_id", type=int,
                        default=id)


    my_experiment = globals()[args.experiment]() #it looks up the value corresponding to the -args.experiment- key in the global symbol table, which includes all global variables and functions. It is expected to be a class or a function.
    map_creation_counter = 0 # counts how many random map have been created

    while True:
        my_sim = my_experiment.create_simulator(args)

        map_rep = save_map_pattern(my_sim.map, my_sim.swarm)

        file_path = str(my_sim.map.pattern_file_path).replace(".txt", "")
        mapName = file_path.replace("benchmarks/", "")
        num_agents = len(my_sim.swarm.agents)

        #print(f'{mapName}')#{num_agents} robots
        if args.create_map and not args.run_experiments and args.num_random_maps > 0:
            if map_creation_counter < args.num_random_maps:
                store_map_txt(MAP_STORAGE_PATH, map_rep)
                map_creation_counter+=1
                continue
            else:
                break


        sim_action = 'start'
        if args.run_experiments:
            pass
        else:
            sim_action = my_sim.start(args)
            map_rep = save_map_pattern(my_sim.map, my_sim.swarm)

        if sim_action == 'reset':
            #logging.info(f"Resetting the Simulation ...")
            continue
        elif sim_action == 'stop':
            break

        sim_action = my_sim.main_loop(args)
        #print(args.create_map)
        if sim_action == None and (not args.create_map or args.run_experiments) : # the simulation ended normally ( without reset or stop )
            export_results(args,
                           my_sim.map,
                           my_sim.swarm,
                           my_sim._step,#step_t
                           my_sim._simulation_time,
                           RESULTS_PATH)
        try:  # run this block if the create_map and run_experiments exist

            if my_sim.swarm.done and args.create_map and not args.run_experiments:  # if swarm done without collisions and create_map is on
                store_map_txt(MAP_STORAGE_PATH, map_rep)
        except AttributeError:
            pass
        if sim_action == 'stop' or args.run_experiments:
            break

# Process class
class Process(multiprocessing.Process):
    """
    class for handling multiprocessing
    """
    def __init__(self, id, arg,map,benchmark:bool=False):
        super(Process, self).__init__()
        self.id = id
        self.arg = arg
        self.map = map
        self.is_benchmark = benchmark

    def run(self):
        #logging.info(f"I'm the process with id: {self.id}...")
        main(self.arg,self.id,self.map,self.is_benchmark)

def choose_map_to_run(arg,maps_folder):
    """
    choose which process runs which map
    """
    maps_choices = {}
    maps_to_run = arg.maps_to_run

    available_maps = []
    if len(maps_to_run)==1 and maps_to_run[0][0] == '0' :
        available_maps = os.listdir(maps_folder)

    elif len(maps_to_run)>1 : # convert to list of integers
        for i in range(0, len(maps_to_run)):
            available_maps.append(f'map_{int(maps_to_run[i][0])}.txt')
    ids = [i for i in range(0, len(available_maps))]
    for map in available_maps:
        id = ids.pop(0)
        maps_choices[id] = os.path.join(maps_folder,map)
    print('maps to run are :',maps_choices)
    return maps_choices


def custom_sort_Benchmarks(file):
    """
     custom sorting function to ensure that the sorting is done based on the letters in each string
    """
    # Remove file extension and split string by '-'
    file_without_extensions = re.sub(r'\.txt$', '', file)
    names = file_without_extensions.split('-')

    names = [word for word in names if not word.isdigit()] # Keep only the words (non-numeric parts)

    return '-'.join(names)

def get_string_between_slashes(s: str) -> str:
    split_str = os.path.normpath(s).split(os.path.sep)
    if len(split_str) >= 2:
        return split_str[1]
    return ''

def extract_number(filename):
    number = re.search(r'-(\d+)\.scen', filename)
    return int(number.group(1)) if number else 0


def get_benchmarks_list(benchmarks_path='benchmarks'):
    """
    This function returns a list of all the files in the benchmarks directo
    """
    benchmark_files  = []
    benchmark_names  = []
    benchmarks_=os.listdir(benchmarks_path)
    new_Benchmark_list = sorted(os.listdir(benchmarks_path), key=custom_sort_Benchmarks)

    for benchmark in new_Benchmark_list:#new_Benchmark_list  #benchmarks_
        if os.path.isfile(os.path.join(benchmarks_path, benchmark)):
            folder = os.path.join(benchmarks_path, benchmark.replace('.txt',''))

            files = os.listdir(folder)  #files are not guaranteed to be in any specific order.
            sorted_files = sorted(files, key=extract_number)
            benchmark_files.append([os.path.join(folder,file) for file in sorted_files])
            benchmark_names.append(os.path.join(benchmarks_path, benchmark))

    #print(benchmark_files)
    return benchmark_files,benchmark_names


def get_benchmark_data(bench_list):
    #print(bench_list)# containe the list of 25 instances
    benchmarks_data = {}
    index = 0

    for benchmark_id, map_params in enumerate(zip(arg.robot_set , bench_list)):  # for every benchmark
        #print(map_params[0])    # the number of robots from arg
        #print(map_params[1][0]) # the map's path

        new_robot_set = []
        mapName = get_string_between_slashes(map_params[1][0])


        if mapName == 'empty-48-48'or mapName == 'warehouse-20-40-10-2-2' or mapName == 'random-64-64-20' :
            new_robot_set = []
            new_robot_set = [50,100, 150, 200,250,300,350, 400, 450]

        elif mapName == 'random-32-32-20':
            new_robot_set= []
            new_robot_set = [50,100,150,200]

        for robot_num in new_robot_set:  # for every robot number in robot_set
            poses_in_maps = []
            for i, m_p in enumerate(map_params[1]):  # for every file in this benchmark folder
                #print(robot_num, m_p)  # the saved data in benchmarks_data{} Dic
                f = open(m_p, "r")
                lines = f.readlines()
                lines.pop(0)
                map = {}
                poses_in_map = []  # store poses of robots in one map
                for line in lines[:robot_num]:  # for every robot
                    line = line.split('\t')
                    poses = [eval(line[4]), eval(line[5]), eval(line[6]), eval(line[7])]
                    poses_in_map.append(poses)
                f.close()
                map['map_file']   = benchs_paths[benchmark_id] # the path of the map.txt
                map['pose_file']  = m_p                        # the path of the scenarios
                map['poses']      = poses_in_map               # starts and goals positions
                map['robots_num'] = robot_num                  # related number of robots
                benchmarks_data[index] = map
                index += 1

    return benchmarks_data,index


if __name__ == "__main__":
    arg = parse_args()
    arg.run_experiments = arg.run_experiments[0]
    arg.batch_size = arg.batch_size[0]
    arg.map = arg.map[0]



    # The path of the results.csv file
    csv_file = 'results_plot/results_data_solvers/DCMAPF/results.csv'

    # The header line
    header = ['map_name', 'map_size', 'obstacles_number', 'num_agents', 'solver', 'solved', 'soc', 'makespan', 'simulation_time']

    # Open the CSV file in write mode, write the header line, and close the file
    #with open(csv_file, 'w', newline='') as csvfile:
    #    csv_writer = csv.writer(csvfile)
    #    csv_writer.writerow(header)

    # Create an empty DataFrame with the specified header
    empty_df = pd.DataFrame(columns=header)

    # Save the empty DataFrame to a CSV file
    empty_df.to_csv(csv_file, index=False)

    if arg.run_experiments:

            if arg.map == 'BenchmarkMapGenerator':
                bench_list,benchs_paths = get_benchmarks_list(BENCHMARK_STORAGE_PATH)

                benchmarks_data,index = get_benchmark_data(bench_list)

                processes = []
                for i in range(0, index // arg.batch_size):
                    processes = []
                    for j in range(arg.batch_size):
                        p_id = j + i * arg.batch_size
                        robotNumber = benchmarks_data[p_id]['robots_num']               # used only to display data
                        Scenario = os.path.basename(benchmarks_data[p_id]['pose_file']) # used only to display data
                        print(f'Running {robotNumber} robots of the scenario {Scenario} ')
                        p = Process(id=p_id, arg=arg, map=benchmarks_data[p_id], benchmark=True)
                        p.start()
                        processes.append(p)

                    for p in processes:
                        p.join()

            else:
                map_to_process = choose_map_to_run(arg, MAP_STORAGE_PATH)
                for i in range(0,len(map_to_process)//arg.batch_size):
                    processes = []
                    for j in range(arg.batch_size):
                        p_id = j + i*arg.batch_size
                        p = Process(id=p_id,arg=arg,map=map_to_process[p_id])
                        p.start()
                        processes.append(p)
                        #logging.info(f'iteration {p_id} is finished')

                    for p  in processes:
                        p.join()

    else:
            main(arg)




