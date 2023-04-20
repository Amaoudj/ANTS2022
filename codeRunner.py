import subprocess
import os

config_file = 'configfiles/experiments_configuration.conf'
target_Algorithm = 'run_experiments.py'
Plotter = 'results_plot/results_plotter.py'



def main():

    subprocess.run(["python3", target_Algorithm, "--config",config_file],stdout=subprocess.PIPE)
    subprocess.run(["python3", Plotter], stdout=subprocess.PIPE)
    print("Simulations completed. Check the results_plot folder for the outcomes of DCMAPF.")

if __name__ == "__main__":
    main()
