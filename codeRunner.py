import subprocess
import os
import re
config_file = 'configfiles/experiments_configuration.conf'
target_Algorithm = 'run_experiments.py'
Plotter = 'results_plot/results_plotter.py'


def custom_sort_Benchmarks(file):
    """
     custom sorting function to ensure that the sorting is done based on the letters in each string
    """
    # Remove file extension and split string by '-'
    file_without_extensions = re.sub(r'\.txt$', '', file)
    names = file_without_extensions.split('-')
    names = [word for word in names if not word.isdigit()] # Keep only the words (non-numeric parts)

    return '-'.join(names)

def main():

    new_Benchmark_list = sorted(os.listdir('benchmarks'), key=custom_sort_Benchmarks)
    new_Benchmark_list = [file for file in new_Benchmark_list if not file.endswith('.txt')]
    print("Running experiments on the following maps : ",new_Benchmark_list)
    print("Please ensure that the number of robots in the <robot_set> variable in the configuration file match the order of the above maps")

    subprocess.run(["python3", target_Algorithm, "--config",config_file],stdout=subprocess.PIPE)
    print("Simulations completed.")
    print(" Plotting the comparative results in terms of success rate and sum-of-costs ...")
    subprocess.run(["python3", Plotter], stdout=subprocess.PIPE)
    print("Done. Check the results_plot folder for the outcomes of DCMAPF.")

if __name__ == "__main__":
    main()
