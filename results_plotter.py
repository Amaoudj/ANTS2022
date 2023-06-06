import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import pandas as pd

pd.set_option("display.max_columns", None)
from matplotlib import pyplot as plt
import os
import sys

RESULTS_FOLDER = "results_plot/results_data_solvers"
folder_path    = "results_plot/plots"

FILTERS = ['map_name']

marker_styles = [
    dict(color='blueviolet', linestyle='dotted', marker='D', markersize=5, mfc='white', linewidth=1),
    dict(color='lawngreen', linestyle='-.', marker='D', markersize=5, mfc='white', linewidth=1),
    dict(color='green', linestyle='-', marker='o', markersize=7, mfc='white'),
    dict(color='blue', linestyle='dotted', marker='o', markersize=5, mfc='white', linewidth=2),
    dict(color='red', linestyle='dashed', marker='D', markersize=5, mfc='white', linewidth=1.5)
    ]

SOLVERS = ['CBS', 'EECBS', 'PIBT', 'PIBT+', 'DCMAPF']#

ROBOT_SET = [[50, 100, 150, 200, 250, 300, 350, 400, 450], [50, 100, 150, 200], [50, 100, 150, 200, 250, 300,350,400, 450], [50, 100, 150, 200, 250, 300, 350, 450]]

MAPS_TO_PLOT = ['empty-48-48.map', 'random-32-32-20.map', 'random-64-64-20.map','warehouse-20-40-10-2-2.map']


def import_results(folder: str) -> dict:
    """
    Given a folder, it will return a dictionary of dataframes, where the key is the solver name and the value is the
    dataframe

    """
    solver_folders = os.listdir(folder)
    solvers_df = {}
    for f in SOLVERS:
        if f in solver_folders:
            dir = os.path.join(folder, f)
            solver = os.listdir(dir)[0]
            df = pd.read_csv(os.path.join(dir, solver))
            df['map_name'] = df['map_name'].str.replace('benchmarks/', '').to_frame()
            df['map_name'] = df['map_name'].str.replace('benchmarks\\\\', '').to_frame()
            df['map_name'] = df['map_name'].str.replace('txt', 'map').to_frame()
            solvers_df[f] = df
            # print(solvers_df)
        else:
            print(f'the solver : {f} results file do not exist ')

    return solvers_df

def filter_by_map(results_df: dict):
    filtered_results = {}
    for solver in results_df:

        filtered_results[solver] = {}
        grouped_solver = results_df[solver].groupby('map_name')
        for group in grouped_solver.groups:
            filtered_results[solver][group] = results_df[solver].loc[grouped_solver.groups[group], :]

    return filtered_results

def clean_dataframe(df, column_names):
    for column_name in column_names:
        df[column_name] = pd.to_numeric(df[column_name], errors='coerce')
    df = df.dropna(subset=column_names)
    return df

def success_rate_and_soc_plot(filtered_res):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    s_rates = {}
    soc_rates = {}

    for solver in filtered_res:
        s_rates[solver] = {}
        soc_rates[solver] = {}
        for map_name in filtered_res[solver]:
            filtered_res[solver][map_name] = clean_dataframe(filtered_res[solver][map_name], ['solved', 'num_agents', 'soc'])

        for i, map in enumerate(MAPS_TO_PLOT):
            try:
                s_rates[solver][map] = {}
                soc_rates[solver][map] = {}
                n_agents_groups = filtered_res[solver][map].groupby('num_agents')

                valid_groups = {int(k): v for k, v in n_agents_groups if int(k) in ROBOT_SET[i]}
                #print(f"For {solver} on {map}, valid groups are: {valid_groups.keys()}")

                for group_key, group_value in valid_groups.items():
                    if solver == 'DCMAPF': #

                        solved_numeric = group_value['solved']
                        successes = solved_numeric.value_counts().get(True, 0)      # Compute the number of solved scenarios, where solved is True

                    else :
                        solved_numeric = pd.to_numeric(group_value['solved'], errors='coerce')
                        successes      = solved_numeric.value_counts().get(1, 0)    # Compute the number of solved scenarios, where solved is 1

                    success_rate   = successes / len(solved_numeric)


                    s_rates[solver][map][group_key] = success_rate
                    soc_numeric = pd.to_numeric(group_value['soc'], errors='coerce')
                    soc_mean = soc_numeric[solved_numeric == 1].mean()
                    soc_rates[solver][map][group_key] = soc_mean

            except KeyError:
                print(f'{map} does not exist in the result file of the folder {solver}')


    for i, map in enumerate(MAPS_TO_PLOT): #map in MAPS_TO_PLOT:
        fig, axs = plt.subplots(1, 2, figsize=(15, 4))
        plt.subplots_adjust(wspace=0.2)  # add space between subplots
        plt.subplots_adjust(left=0.05)  #,right=1 adjust this line to remove space on the left and right

        legend_list = []

        for id, solver in enumerate(s_rates):
            try:
                x_axis = s_rates[solver][map].keys()
                y_axis = s_rates[solver][map].values()
                axs[0].plot(x_axis, y_axis, **marker_styles[id])
                legend_list.append(solver)

                y_axis_soc = soc_rates[solver][map].values()
                axs[1].plot(x_axis, y_axis_soc, **marker_styles[id])

            except KeyError:
                print(f'{map} do not exist in the solver : {solver}')

        # set x-axis to match the values defined in ROBOT_SET[i]
        axs[0].set_xticks(ROBOT_SET[i])
        axs[1].set_xticks(ROBOT_SET[i])

        axs[0].set_ylim(bottom=0, top=1.05)
        axs[0].set_ylabel('Success rate', fontsize=13)

        axs[1].set_ylabel('Sum-of-costs', fontsize=13)

        axs[0].set_xlabel('Number of robots', fontsize=13)
        axs[1].set_xlabel('Number of robots', fontsize=13)

        # set legend for each subplot to contain all algorithms
        if map == 'random-32-32-20.map':
            # set legend for each subplot to contain all algorithms and place it at the middle bottom
            axs[0].legend(legend_list, loc='lower center', bbox_to_anchor=(0.5, 0), fontsize=12)
        else:
            axs[0].legend(legend_list, fontsize=12)

        axs[1].legend(legend_list, fontsize=12)

        #plt.suptitle(f'{map[:-4]}')

        title = f'{map[:-4]}.pdf'
        file_path = os.path.join(folder_path, title)
        plt.savefig(file_path, format='pdf')
        plt.close(fig)


def main():

   data = import_results(RESULTS_FOLDER)
   filtered_res = filter_by_map(data)
   success_rate_and_soc_plot(filtered_res)


if __name__ == "__main__":
    main()

