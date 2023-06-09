import logging
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import os
pd.set_option("display.max_rows", None, "display.max_columns", None)
# The StatisticsPlotter class is a class that takes in a data path and has a function that plots the statistics of the
# data
class StatisticsPlotter:
    def __init__(self, data_path:str):
        self.data_path = data_path
        self.x,self.y = None,None
        self.data=pd.read_csv(self.data_path)

    def plot(self,
             x: list,
             y: list,
             categorical_vars=None,
             show: bool = False,
             save: bool = True,
             save_path: str = 'conf_experiments'):
        """
        It plots the data.

        :param x: The name of the columns in the dataframe that you want to plot on the x-axis
        :type x: list
        :param y: list The name of the columns in the dataframe that you want to plot on the y-axis
        :type y: list
        :param show: bool  If True the plot will be shown defaults to False
        :type show: bool (optional)
        :param save: If True, the plot will be saved to the specified path, defaults to True
        :type save: bool (optional)
        :param save_path: The path to save the plot to, defaults to conf_experiments/fig.png
        :type save_path: str (optional)
        """
        title = None
        if y[0] == 'success_rate':
            title = self.plot_data_srate(self.data, x_axis=x[0], cat=categorical_vars[0])
            if save:
                plt.savefig(os.path.join(save_path, title))
            if show:
                plt.show()
        else:

            ax = self.plot_data(self.data,x_axis=x[0],y_axis=y[0])
            fig = ax.get_figure()
            y_label = y[0].replace("_", " ")
            x_label = x[0].replace("_", " ")
            title = f'{y_label} in function of {x_label}'
            plt.xlabel(x_label)
            plt.ylabel(y_label)
            ax.get_legend().remove()
            if save:

                fig.savefig(os.path.join(save_path, title))
            if show:
                plt.show()



    def process_data(self,data_frame:pd.DataFrame,x_axis):
        """
        This function takes in a dataframe and two column names, and returns a sorted dataframe

        :param data_frame: the data frame to be processed
        :type data_frame: pd.DataFrame

        """
        print(x_axis)
        data = data_frame.sort_values(x_axis,ascending=True)
        #self.group_data_by(data,None)
        return data

    def plot_data(self,dataframe,x_axis,y_axis):
        print(dataframe)
        data = self.process_data(dataframe,x_axis)
        ax = data.plot(x=x_axis,y=y_axis)
        return ax


    def success_rate(self,dataframe:pd.DataFrame):
        """
        This function takes in a dataframe and returns the success rate
        :param dataframe: the dataframe to be used
        :type dataframe: pd.DataFrame
        :return: A percentage of the number of successful experiments over the total number of experiments.
        """

        if True in dataframe['is_done'].value_counts().to_dict().keys():
            s_rate = dataframe['is_done'].value_counts()[True]/len(dataframe['is_done']) * 100
            return s_rate
        else:
            return  0

    def add_obstacles_density(self,dataframe:pd.DataFrame):
        """
        :param dataframe: the dataframe to which we want to add the obstacle density
        :type dataframe: pd.DataFrame
        :return: The dataframe with the new column added.
        """
        dataframe['obstacles_density'] = dataframe['obstacles_number']/dataframe['map_size'] *100
        return  dataframe.round(2)

    def group_data_by(self,data:pd.DataFrame,cat:str)->tuple:
        """
        Given a dataframe and a categorical variable, the function groups the dataframe by the categorical variable and
        returns a list of dataframes and a list of group_ids

        :param data: The dataframe you want to group
        :type data: pd.DataFrame
        :param cat: The name of the column that you want to group by
        :type cat: str
        :return: A tuple of two elements. The first element is a list of dataframes. The
        second element is a list of group ids.
        """
        groupedby = data.groupby(cat)
        data_groups = []
        for group_id in groupedby.groups.keys():
            data_groups.append(self.data.loc[groupedby.groups[group_id],:])
        return  data_groups,groupedby.groups.keys()

    def plot_data_srate(self, dataframe:pd.DataFrame,
                        x_axis:str='agents_number',
                        cat:str='obstacle_density'):
        """
        :param dataframe: the dataframe containing the data to be plotted
        :type dataframe: pd.DataFrame
        :param x_axis: the x-axis of the plot, defaults to
        agents_number
        :type x_axis: str (optional)
        :param cat: the category you want to plot, defaults to obstacle_density
        :type cat: str (optional)
        """

        marker_style = dict(color='blueviolet', linestyle='-', marker='D',
                            markersize=5, mfc='white')
        s_rate_list = []
        x_axis_list  = []
        title = None
        if cat == 'None':
            x_axis_groups, keys = self.group_data_by(dataframe, x_axis)
            s_rate_list.append([self.success_rate(group) for group in x_axis_groups])
            x_axis_list.append(keys)


            title = f'success rate in function of {x_axis.replace("_", " ")}'
            plt.xlabel(x_axis.replace("_", " "))
            plt.ylabel('success rate')
            plt.ylim(bottom=0,top=1.05)
            for i,rate in enumerate(s_rate_list):
                plt.plot(x_axis_list[i],[i/100 for i in rate],**marker_style)
        else:
            dataframe = self.add_obstacles_density(dataframe)
            density_groups,densities = self.group_data_by(dataframe,cat)
            s_rate_list = []
            x_axis_list  = []
            for group in density_groups:
                x_axis_groups,keys = self.group_data_by(group,x_axis)
                s_rate_list.append ([self.success_rate(group) for group in x_axis_groups])
                x_axis_list.append (keys)

            title = f'success rate in function of {x_axis.replace("_", " ")} for different densities'
            plt.xlabel(x_axis.replace("_", " "))
            plt.ylabel('success rate')
            plt.autoscale()
            plt.ylim(bottom=0,top=1.05)

            for i,rate in enumerate(s_rate_list):
                plt.plot(x_axis_list[i],[i/100 for i in rate])

            plt.legend([f'obstacles density : {i} %' for i in densities])

        plt.title(title)
        return title
