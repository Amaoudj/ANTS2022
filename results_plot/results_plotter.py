import warnings
import numpy as np

warnings.simplefilter(action='ignore', category=FutureWarning)
import pandas as pd
pd.set_option("display.max_columns", None)

from matplotlib import pyplot as plt
import  os


RESULTS_FOLDER = "../swarm4I40sim/results_plot/results_data_solvers"
FILTERS = ['map_name']
scen_num= 18 # not used
marker_styles = [

                 dict(color='blueviolet', linestyle='dotted', marker='D', markersize=5, mfc='white', linewidth=1),
                 dict(color='lawngreen', linestyle='-.', marker='D',markersize=5, mfc='white',linewidth=1),
                 dict(color='green', linestyle='-', marker='o',markersize=7, mfc='white'),
                 dict(color='blue', linestyle='dotted', marker='o',markersize=5, mfc='white',linewidth=2),
                 dict(color='red', linestyle='dashed', marker='D',markersize=5, mfc='white',linewidth=1.5)
               ]
SOLVERS = ['CBS','EECBS','PIBT','PIBT+','DCMAPF']
ROBOT_SET = [[50,100,150,200,250,300,350,400,450],[50,100,150,200],[50,100,150,200,250,300,350,400,450],[50,100,150,200,250,300,350,400,450]]
MAPS_TO_PLOT = ['empty-48-48.map','random-32-32-20.map','random-64-64-20.map','warehouse-20-40-10-2-2.map']

def import_results(folder:str)->dict:
    """
    Given a folder, it will return a dictionary of dataframes, where the key is the solver name and the value is the
    dataframe
    :param folder: the folder where the solvers results are stored
    :type folder: str
    :return: A dictionary of dataframes.
    """
    solver_folders = os.listdir(folder)
    solvers_df = {}
    for f in SOLVERS:
        if f in solver_folders:
            dir = os.path.join(folder,f)
            solver = os.listdir(dir)[0]
            df = pd.read_csv(os.path.join(dir,solver))
            df['map_name'] = df['map_name'].str.replace('benchmarks\\\\','').to_frame()
            df['map_name'] = df['map_name'].str.replace('txt','map').to_frame()
            solvers_df[f] = df

        else:
            print(f'the solver : {f} results file do not exist ')

    return solvers_df

def filter_by_map(results_df:dict):
    filtered_results = {}
    for solver in results_df:

        filtered_results[solver] = {}
        grouped_solver = results_df[solver].groupby('map_name')
        for group in grouped_solver.groups:
            filtered_results[solver][group]=results_df[solver].loc[grouped_solver.groups[group],:]

    return filtered_results


def sucess_rate_plot(filtered_res):
    s_rates = {}

    for solver in filtered_res:
        s_rates[solver] = {}

        for i,map in enumerate(MAPS_TO_PLOT):

            try:
                s_rates[solver][map] = {}

                n_agents_groups = filtered_res[solver][map].groupby('num_agents')

                for group in n_agents_groups:
                    if group[0] in ROBOT_SET[i]:
                        s_rate = 0
                        if 1 in  group[1]['solved'].to_list() or True in group[1]['solved'].to_list():
                            s_rate = group[1]['solved'].value_counts()[1] / len(group[1]['solved']) * 100
                        s_rates[solver][map][group[0]]=s_rate/100.0
                    else:
                        pass
            except KeyError:
                print(f'{map} do not exist in the result file of the folder {solver}')

    for map in MAPS_TO_PLOT:
        legend_list = []
        for id,solver in enumerate(s_rates):
            try:
                x_axis = s_rates[solver][map].keys()
                y_axis = s_rates[solver][map].values()
                plt.plot(x_axis,y_axis,**marker_styles[id])
                legend_list.append(solver)
            except KeyError:
                print(f'{map} do not exist in the solver : {solver}')

        title = f'success rate in {map[:-4]}'
        #plt.title(title)

        plt.ylim(bottom=0, top=1.05)
        plt.legend(legend_list, fontsize=13)
        plt.xlabel('Number of robots', fontsize=13)
        plt.ylabel('Success rate', fontsize=13)
        plt.savefig(title)
        plt.show()


def general_plot(filtered_res,axis,scen_num=np.nan,save_path:str='results_plot'):
    x_axis_name,y_axis_name = axis

    for i,map in enumerate(MAPS_TO_PLOT):
        legend_list = []
        for id,solver in enumerate(filtered_res):
            x_axis_val, y_axis_val = [], []
            try:
                if 'scen_num' not in filtered_res[solver][map]:
                    filtered_res[solver][map]['scen_num'] = scen_num

                #if scen_num != np.nan:
                    #filtered_res[solver][map] = filtered_res[solver][map][filtered_res[solver][map]['scen_num'] == scen_num]

                if -1 in filtered_res[solver][map]['solved'].to_list() :
                   filtered_res[solver][map] = filtered_res[solver][map][filtered_res[solver][map]['solved'] != -1 ]

                elif 0 in filtered_res[solver][map]['solved'].to_list() :
                    filtered_res[solver][map] = filtered_res[solver][map][filtered_res[solver][map]['solved'] != 0]

                elif False in filtered_res[solver][map]['solved'].to_list() :
                    filtered_res[solver][map] = filtered_res[solver][map][filtered_res[solver][map]['solved'] != False]

                print(filtered_res[solver][map]['solved'])
                print(f'-----------------------------------------------')
                n_agents_groups = filtered_res[solver][map].groupby('num_agents')
                for group in n_agents_groups:
                    if group[0] in ROBOT_SET[i]:
                        y_axis_val.append(group[1].mean()[y_axis_name])
                        x_axis_val.append(group[0])
                    else:
                        pass
                legend_list.append(solver)
                plt.plot(x_axis_val,y_axis_val,**marker_styles[id])
            except KeyError:
                print(f'{map} do not exist in the solver : {solver}')
        title = f'{y_axis_name.replace("_", " ")} in {map[:-4]}'
        #-----------------------
        plt.legend(legend_list, fontsize=13)
        plt.xlabel('Number of robots', fontsize=13)
        plt.ylabel('Sum-of-costs', fontsize=13)

        plt.savefig(title + ".pdf")
        plt.savefig(title)

        #plt.show()


r = import_results(RESULTS_FOLDER)
filtered_res = filter_by_map(r)


sucess_rate_plot(filtered_res)
general_plot(filtered_res,['num_agents','soc'],scen_num=scen_num,save_path="results_plot")


