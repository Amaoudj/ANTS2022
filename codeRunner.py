import subprocess
import os
import re

config_file = 'configfiles/experiments_configuration.conf'
target_Algorithm = 'run_experiments.py'
Plotter = 'results_plotter.py'


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
    print("Running experiments on the following maps : ", new_Benchmark_list)
    with subprocess.Popen(["python3", target_Algorithm, "--config", config_file], stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True, bufsize=1, universal_newlines=True) as proc:
        for line in proc.stdout:
            print(line, end='')
        for line in proc.stderr:
            print(line, end='')

    returncode = proc.returncode

    print("----------------------------------------------")
    print("           < Simulations completed >")
    print("----------------------------------------------")
    print("Check the results.csv file in results_plots >> results_data_solvers, for the outcomes of DCMAPF.")
    print("")
    print("")
    print("Plotting the comparative results in terms of success rate and sum-of-costs ...")
    print("")
    subprocess.run(["python3", Plotter], stdout=subprocess.PIPE)
    print("Done. Check the <plots> folder in <results_plots> for the outcomes of DCMAPF.")

if __name__ == "__main__":
    main()
