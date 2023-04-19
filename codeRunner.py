#!/usr/bin/env python3

import sys
import json
import argparse
import subprocess
import os
import re
import csv
import random


current_directory = os.path.dirname(os.path.abspath(__file__))
config_file = os.path.join(current_directory, "../swarm4I40sim/configfiles/experiments_configuration.conf")
target_Algorithm = os.path.join(current_directory, "../swarm4I40sim/swarmI4/run_experiments.py")
Plotter = os.path.join(current_directory, "../swarm4I40sim/results_plot/results_plotter.py")

def target_runner():


  run1 =subprocess.run(["python3", target_Algorithm, "--config",config_file],stdout=subprocess.PIPE)
  run2 = subprocess.run(["python3", Plotter], stdout=subprocess.PIPE)


def main():
   target_runner()
   print("Simulations completed. Check the results_plot folder for the outcomes of DCMAPF.")

if __name__ == "__main__":
    main()
