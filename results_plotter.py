import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import pandas as pd

pd.set_option("display.max_columns", None)
from matplotlib import pyplot as plt
import os
import sys

RESULTS_FOLDER = "results_plot/results_data_solvers"
folder_path = "results_plot/plots"

FILTERS = ['map_name']

marker_styles = [
    dict(color='blueviolet', linestyle='dotted', marker='D', markersize=5, mfc='white', linewidth=1),
    dict(color='lawngreen', linestyle='-.', marker='D', markersize=5, mfc='white', linewidth=1),
    dict(color='green', linestyle='-', marker='o', markersize=7, mfc='white'),
    dict(color='blue', linestyle='dotted', marker='o', markersize=5, mfc='white', linewidth=2),
    dict(color='red', linestyle='dashed', marker='D', markersize=5, mfc='white', linewidth=1.5)
    ]

SOLVERS = ['CBS', 'EECBS', 'PIBT', 'PIBT+', 'DCMAPF']

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



def success_rate_plot(filtered_res):

    # Create the plots folder if it does not exist
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    s_rates = {}
    for solver in filtered_res:
        s_rates[solver] = {}
        for map_name in filtered_res[solver]:
            filtered_res[solver][map_name] = clean_dataframe(filtered_res[solver][map_name], ['solved', 'num_agents'])

        for i, map in enumerate(MAPS_TO_PLOT):

            try:
                s_rates[solver][map] = {}
                n_agents_groups = filtered_res[solver][map].groupby('num_agents')

                for group in n_agents_groups:
                    g0  = pd.to_numeric(group[0], errors='coerce')

                    if g0 in ROBOT_SET[i]:#group[0]
                        s_rate = 0

                        # Filter out non-numeric values using pd.to_numeric()
                        solved_numeric = pd.to_numeric(group[1]['solved'], errors='coerce')

                        if 1 in solved_numeric.to_list() or True in solved_numeric.to_list():
                        #if 1 in group[1]['solved'].to_list() or True in group[1]['solved'].to_list():
                            s_rate = solved_numeric.mean() * 100

                        s_rates[solver][map][g0] = s_rate / 100.0  #group[0]
                    else:
                        pass
            except KeyError:
                print(f'{map} does not exist in the result file of the folder {solver}')

    for map in MAPS_TO_PLOT:
        legend_list = []
        for id, solver in enumerate(s_rates):
            try:
                x_axis = s_rates[solver][map].keys()
                y_axis = s_rates[solver][map].values()
                plt.plot(x_axis, y_axis, **marker_styles[id])
                legend_list.append(solver)
            except KeyError:
                print(f'{map} do not exist in the solver : {solver}')


        plt.ylim(bottom=0, top=1.05)
        plt.legend(legend_list, fontsize=13)
        plt.xlabel('Number of robots', fontsize=13)
        plt.ylabel('Success rate', fontsize=13)

        title = f'Success rate in {map[:-4]}.png'
        file_path =  os.path.join(folder_path, title)
        plt.savefig(file_path)
        plt.show()
        fig = plt.gcf()  # Get the current figure
        plt.close(fig)  # Close the figure


def general_plot(filtered_res, axis):
    x_axis_name, y_axis_name = axis

    for solver in filtered_res:
        for map_name in filtered_res[solver]:
            filtered_res[solver][map_name] = clean_dataframe(filtered_res[solver][map_name], ['solved', 'num_agents', 'soc'])

    for i, map in enumerate(MAPS_TO_PLOT):
        legend_list = []
        for id, solver in enumerate(filtered_res):
            x_axis_val, y_axis_val = [], []
            try:

                # Filter out non-numeric values using pd.to_numeric()
                solved_numeric = pd.to_numeric(filtered_res[solver][map]['solved'], errors='coerce')

                #if -1 in filtered_res[solver][map]['solved'].to_list():
                if -1 in solved_numeric.to_list():
                    filtered_res[solver][map] = filtered_res[solver][map][solved_numeric != -1]

                #elif 0 in filtered_res[solver][map]['solved'].to_list():
                elif 0 in solved_numeric.to_list():
                    filtered_res[solver][map] = filtered_res[solver][map][solved_numeric != 0]

                #elif False in filtered_res[solver][map]['solved'].to_list():
                elif False in solved_numeric.to_list():
                    filtered_res[solver][map] = filtered_res[solver][map][solved_numeric != False]

                #print(f'-----------------------------------------------')

                n_agents_groups = filtered_res[solver][map].groupby('num_agents')
                #solved_numeric = pd.to_numeric(n_agents_groups, errors='coerce')

                for group in n_agents_groups: #solved_numeric.to_list()
                    G0=pd.to_numeric(group[0], errors='coerce')
                    G1=pd.to_numeric(group[1].mean()[y_axis_name], errors='coerce')

                    if G0 in ROBOT_SET[i]: #group[0]
                        y_axis_val.append(G1)#
                        x_axis_val.append(G0)#group[0]

                    else:
                        pass
                legend_list.append(solver)
                plt.plot(x_axis_val, y_axis_val, **marker_styles[id])
            except KeyError:
                print(f'{map} do not exist in the solver : {solver}')
        title = f'{y_axis_name.replace("_", " ")} in {map[:-4]}.png'

        # -----------------------
        plt.legend(legend_list, fontsize=13)
        plt.xlabel('Number of robots', fontsize=13)
        plt.ylabel('Sum-of-costs', fontsize=13)
        file_path =  os.path.join(folder_path, title)
        plt.savefig(file_path)
        plt.show()
        fig = plt.gcf()  # Get the current figure
        plt.close(fig)  # Close the figure


def general_plot2(filtered_res, axis):
    x_axis_name, y_axis_name = axis

    for i, map in enumerate(MAPS_TO_PLOT):
        legend_list = []
        for id, solver in enumerate(filtered_res):
            x_axis_val, y_axis_val = [], []
            try:

                if -1 in filtered_res[solver][map]['solved'].to_list():
                    filtered_res[solver][map] = filtered_res[solver][map][filtered_res[solver][map]['solved'] != -1]

                elif 0 in filtered_res[solver][map]['solved'].to_list():
                    filtered_res[solver][map] = filtered_res[solver][map][filtered_res[solver][map]['solved'] != 0]

                elif False in filtered_res[solver][map]['solved'].to_list():
                    filtered_res[solver][map] = filtered_res[solver][map][filtered_res[solver][map]['solved'] != False]

                n_agents_groups = filtered_res[solver][map].groupby('num_agents')

                for group in n_agents_groups:
                    if group[0] in ROBOT_SET[i]:
                        G_numeric = pd.to_numeric(group[1].mean()[y_axis_name], errors='coerce')
                        y_axis_val.append(G_numeric)
                        x_axis_val.append(group[0])
                    else:
                        pass
                legend_list.append(solver)
                plt.plot(x_axis_val, y_axis_val, **marker_styles[id])
            except KeyError:
                print(f'{map} do not exist in the solver : {solver}')
        title = f'{y_axis_name.replace("_", " ")} in {map[:-4]}.png'

        # -----------------------
        plt.legend(legend_list, fontsize=13)
        plt.xlabel('Number of robots', fontsize=13)
        plt.ylabel('Sum-of-costs', fontsize=13)
        file_path = os.path.join(folder_path, title)
        plt.savefig(file_path)
        plt.show()
        fig = plt.gcf()  # Get the current figure
        plt.close(fig)  # Close the figure


def remove_empty_lines(input_file):
    df = pd.read_csv(input_file)
    df.dropna(how='all', inplace=True)  # Remove rows with all missing values
    df.to_csv(input_file, index=False)

def remove_last_empty_line(input_file):
    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(input_file)
    # Remove any rows with all missing values
    df.dropna(how='all', inplace=True)
    # Write the cleaned DataFrame back to the original file
    df.to_csv(input_file, index=False, encoding='utf-8')


def remove_string_from_csv(input_file):
    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(input_file)
    target_string = 'CBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBSCBS'

    # Replace the target_string with an empty string in the entire DataFrame
    df.replace(target_string, '', inplace=True, regex=True)

    # Write the cleaned DataFrame back to the input file
    df.to_csv(input_file, index=False, encoding='utf-8')


def main():

   remove_empty_lines( "results_plot/results_data_solvers/DCMAPF/results.csv")
   #remove_string_from_csv("results_plot/results_data_solvers/DCMAPF/results.csv")

   data = import_results(RESULTS_FOLDER)
   filtered_res = filter_by_map(data)
   #success_rate_plot(filtered_res)
   general_plot(filtered_res,['num_agents','soc'])

if __name__ == "__main__":
    main()

