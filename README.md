# Swarm Robotics for I4.0 Simple Simulator

This simulator is developed as part of the "Swarm Robotics for I4.0" research project. The goal is to enable the simulation on embodied agents operating in a discrete world. The world is a graph where nodes are locations and edges denote connectivity between locations.

# Getting started

From a terminal:

```sh
git clone https://github.com/anderslyhne/swarm4I40sim.git

cd swarm4I40sim
```
To run an example from the terminal:

```sh
python -m swarmI40 -c configfiles/example.conf
```

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


