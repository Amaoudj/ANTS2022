#!/usr/bin/python
import argparse
import glob,os
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost
from sipp_independent import SIPP_IndependentSolver
from sipp_cbs import SIPP_CBSSolver
from graph_generation import SippGraph

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)

def cost_sum(paths):
    val = 0
    for path in paths:
       val += len(path)
    return val,len(path) - 1


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)

def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@' or cell == 'T':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


if __name__ == '__main__':
    #parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    #parser.add_argument('--instance', type=str, default=None,help='The name of the instance file(s)')
    #parser.add_argument('--batch', action='store_true', default=False,help='Use batch output instead of animation')
    #parser.add_argument('--disjoint', action='store_true', default=False,help='Use the disjoint splitting')
    #parser.add_argument('--solver', type=str, default=SOLVER,help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))
    #args = parser.parse_args()
    result_file = open("results.csv", "w", buffering=1)


    SOLVER_ = "prioritized"  #cbs,independent,prioritized,sipp_cbs,sipp_independent
    instance = 'instances/test_47.txt'

    disjoint_cbs      =False
    display_simulation=True

    #for file in sorted(glob.glob(args.instance)):
    if os.path.isfile(instance):
        print("\n\n*********************************************************")
        print("***Import an instance***")
        print(instance, end = '\n\n')
        my_map, starts, goals = import_mapf_instance(instance)
        print_mapf_instance(my_map, starts, goals)
        cost = 0

        if SOLVER_== "cbs":#args.solver
            print("***Run CBS***\n")
            cbs = CBSSolver(my_map, starts, goals)
            paths = cbs.find_solution(disjoint_cbs, CBSSolver.NORMAL)

        elif SOLVER_ == "independent":
            print("***Run Independent***\n")
            solver = IndependentSolver(my_map, starts, goals)
            paths = solver.find_solution()
        elif SOLVER_ == "prioritized":
            print("***Run Prioritized***\n")
            solver = PrioritizedPlanningSolver(my_map, starts, goals)
            paths = solver.find_solution()

        elif SOLVER_ == "sipp_independent":
            print("***Run SIPP Independent***")
            my_map = SippGraph(instance, None)
            solver = SIPP_IndependentSolver(instance, my_map, starts, goals)
            paths = solver.find_solution()
            for path in paths:
                cost += len(path)
            _,cost_= cost_sum(paths)
            print("Sum of Costs: ",cost_ )
            print("### PATHS ###")
            print(paths)
        elif SOLVER_ == "sipp_cbs":
            print("***Run SIPP CBS***")
            my_map = SippGraph(instance, None)
            solver = SIPP_CBSSolver(instance, my_map, starts, goals)
            paths = solver.find_solution()

            print("Sum of Costs: ", )
            print(paths)
            print("CPU Time:       {:.2f}".format(solver.CPU_time))
            _, cost_ = cost_sum(paths)
            print("Sum of Costs: ", cost_)
        else:
            raise RuntimeError("Unknown solver!")

        _,cost = get_sum_of_cost(paths)
        result_file.write("{},{}\n".format(instance, cost))


        if display_simulation:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            # animation.save("warehouse.mp4", 1.0)
            animation.show()

    result_file.close()
