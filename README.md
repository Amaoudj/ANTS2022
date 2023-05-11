# Decentralized Multi-Agent Path Finding in Warehouse Environments for Fleets of Mobile Robots with Limited Communication Range

This simulator has been developed as part of the 'Swarm Robotics for I4.0' research project. The work features a decentralized multi-agent path-finding approach for mobile robots with a limited communication range, known as DCMAPF. In the proposed DCMAPF planner,
each robot plans its shortest path offline, and then autonomously coordinates with its neighbors to solve potential conflicts as they occur during task execution. 
# Getting started

From a terminal:

```sh
git clone https://github.com/Amaoudj/ANTS2022.git

cd ANTS2022
```
Add the maps along with their scenario files to the 'benchmarks' folder. Note that the folder containing the scenarios of a map should have the same name as the map. Also, it is important that the maps are formatted according to the standards provided at https://movingai.com/benchmarks/mapf/index.html.".

In the configuration file located at 'configfiles/experiments_configuration.conf', adjust the number of processes you can run in parallel "batch_size", according to your computer's specifications, to either 1, 5, or 25.

To run experiments and plot the results from the terminal:

```sh
python3 codeRunner.py
```
To run only experiments from the terminal

```sh
python3 run_experiments.py -c configfiles/experiments_configuration.conf
```

The results can be found in the folder 'results_plot/results_data_solvers/DCMAPF/results.csv'.

The plots can be found in the folder 'results_plot/plots'.

# Using pycharm in docker
Build the docker image (install docker.io if not already installed `sudo apt install docker.io`):

```sh
make build
```

Make and start a docker container with the image created above:

```sh 
make start
```

You should now be in a shell in the directory called `~/package` in the container.

To use PyCharm as an IDE, do the following:
```
pycharm.sh
```

When PyCharm starts the first time, you have to accept the licence agreement and "Open" the project. Open the directory  "package".

In the lower right corner in the PyCharm window, you'll find the text "&lt;No interpreter&gt;". Click on that and select "Add interpreter". A window pops up. Select "System Interpreter" and then in the "Interpreter" select "/usr/bin/python3.8".

You are good to go! To test, go to "Run" and "Run __main__" 


# Credit

The simulation code and the Makefile/docker setup is based on https://github.com/TheWorldOfCode/project-in-swarms by Dan Nykjær Jakobsen <dannj75@gmail.com>. Via email, the author informed me that the code should be distributed under the BSD-3 licence.


