# Decentralized Multi-Agent Path Finding in Warehouse Environments for Fleets of Mobile Robots with Limited Communication Range

This simulator has been developed as part of the 'Swarm Robotics for I4.0' research project. The work features a decentralized multi-agent path-finding approach for mobile robots with a limited communication range, known as DCMAPF. In the proposed DCMAPF planner,
each robot plans its shortest path offline, and then autonomously coordinates with its neighbors to solve potential conflicts as they occur during task execution. 
# Getting started

From a terminal:

```sh
git clone https://github.com/Amaoudj/ANTS2022.git

cd ANTS2022
```
Add the maps along with their scenario files to the 'benchmarks' folder. Note that the folder containing the scenarios of a map should have the same name as the map.

In the configuration file located at 'configfiles/experiments_configuration.conf', adjust the number of processes you can run in parallel "batch_size", according to your computer's specifications, to either 1, 5, or 25.

To run experiments and plot the results from the terminal:

```sh
python3 codeRunner.py
```


The results can be found in the folder 'results_plot/results_data_solvers/DCMAPF/results.csv'.

The plots can be found in the folder 'results_plot/plots'.


# Credit

The simulation code is based on https://github.com/TheWorldOfCode/project-in-swarms by Dan Nykjær Jakobsen <dannj75@gmail.com>. Via email, the author informed me that the code should be distributed under the BSD-3 licence.


