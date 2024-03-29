import os
import json
import glob

import pandas as pd
# import matplotlib.pyplot as plt
# import seaborn as sns


def ip_style_to_name(ip_style):
    if ip_style == "kf":
        return "Keyframe Only"
    if ip_style == "opt":
        return "Optimization Only"
    if ip_style == "optkf":
        return "Optimization + Keyframe"

def planning_bias_to_name(planning_bias):
    if planning_bias == "1":
        return "Biased Planning"
    if planning_bias == "0":
        return "Unbiased Planning"

def participant_to_name(participant):
    return "Participant {}".format(participant)

def import_data_as_dataframe(data_path):
    path = os.path.join(data_path, "*.json")
    files_names = glob.glob(path)
    dfs = []
    for filene_name in files_names:
        with open(filene_name, 'r') as f:
            data = json.loads(f.read())
            dfs.append(pd.json_normalize(data, record_path=['trials'], meta=[
                'participant',
                'planning_bias',
                'ip_style'
            ]))
    return pd.concat(dfs)


class PlanningTimeAnalysis():

    def __init__(self, data_directory):
        self.data_directory = data_directory
        self.dataframe = self._import_data()


    # def bar_chart(self):

    #     df = self.dataframe[["participant", "planning_bias", "ip_style", "planning_time"]]
    #     df = df.set_index(['participant', 'planning_bias', 'ip_style'])     
    #     # plot dfl
    #     ax = sns.barplot(data=df)  # RUN PLOT   
    #     plt.legend(bbox_to_anchor=(1.04, 0.5), loc="center left", borderaxespad=0)
    #     plt.close()
        
    def stats(self):
        successful_df = self.dataframe[self.dataframe['success'] == True]
        print(successful_df)
        df = successful_df[["participant", "planning_bias", "ip_style", "planning_time"]]
        mean = df.groupby(['planning_bias', 'ip_style']).mean()
        std = df.groupby(['planning_bias', 'ip_style']).std()
        return mean, std

    
    def _import_data(self):
        return import_data_as_dataframe(self.data_directory)
    

class SuccessPercentageAnalysis():

    def __init__(self, data_directory):
        self.data_directory = data_directory
        self.dataframe = self._import_data()


    # def bar_chart(self):

    #     df = self.dataframe[["participant", "planning_bias", "ip_style", "planning_time"]]
    #     df = df.set_index(['participant', 'planning_bias', 'ip_style'])     
    #     # plot dfl
    #     ax = sns.barplot(data=df)  # RUN PLOT   
    #     plt.legend(bbox_to_anchor=(1.04, 0.5), loc="center left", borderaxespad=0)
    #     plt.close()
        
    def stats(self):
        self.dataframe['success'] = self.dataframe['success'].replace(['X'], False)
        df = self.dataframe[["participant", "planning_bias", "ip_style", "success"]]
        success_df = df.groupby(['planning_bias', 'ip_style'])['success'].mean()
        success_percentage = success_df[success_df > 0.1]
        return success_percentage

    
    def _import_data(self):
        return import_data_as_dataframe(self.data_directory)
    
class PathLengthAnalysis():

    def __init__(self, data_directory):
        self.data_directory = data_directory
        self.dataframe = self._import_data()


    # def bar_chart(self):

    #     df = self.dataframe[["participant", "planning_bias", "ip_style", "path_length"]]
    #     df = df.set_index(['participant', 'planning_bias', 'ip_style'])     
    #     # plot dfl
    #     ax = sns.barplot(data=df)  # RUN PLOT   
    #     plt.legend(bbox_to_anchor=(1.04, 0.5), loc="center left", borderaxespad=0)
    #     plt.close()
        
    def stats(self):
        successful_df = self.dataframe[self.dataframe['success'] == True]
        df = successful_df[["participant", "planning_bias", "ip_style", "path_length"]]
        mean = df.groupby(['planning_bias', 'ip_style']).mean()
        std = df.groupby(['planning_bias', 'ip_style']).std()
        return mean, std

    
    def _import_data(self):
        return import_data_as_dataframe(self.data_directory)
    
class A2SAnalysis():

    def __init__(self, data_directory):
        self.data_directory = data_directory
        self.dataframe = self._import_data()


    # def bar_chart(self):

    #     df = self.dataframe[["participant", "planning_bias", "ip_style", "path_length"]]
    #     df = df.set_index(['participant', 'planning_bias', 'ip_style'])     
    #     # plot dfl
    #     ax = sns.barplot(data=df)  # RUN PLOT   
    #     plt.legend(bbox_to_anchor=(1.04, 0.5), loc="center left", borderaxespad=0)
    #     plt.close()
        
    def stats(self):
        successful_df = self.dataframe[self.dataframe['success'] == True]
        df = successful_df[["participant", "planning_bias", "ip_style", "a2s_distance"]]
        mean = df.groupby(['planning_bias', 'ip_style']).mean()
        std = df.groupby(['planning_bias', 'ip_style']).std()
        return mean, std

    
    def _import_data(self):
        return import_data_as_dataframe(self.data_directory)
    
class A2FAnalysis():

    def __init__(self, data_directory):
        self.data_directory = data_directory
        self.dataframe = self._import_data()


    # def bar_chart(self):

    #     df = self.dataframe[["participant", "planning_bias", "ip_style", "path_length"]]
    #     df = df.set_index(['participant', 'planning_bias', 'ip_style'])     
    #     # plot dfl
    #     ax = sns.barplot(data=df)  # RUN PLOT   
    #     plt.legend(bbox_to_anchor=(1.04, 0.5), loc="center left", borderaxespad=0)
    #     plt.close()
        
    def stats(self):
        successful_df = self.dataframe[self.dataframe['success'] == True]
        df = successful_df[["participant", "planning_bias", "ip_style", "a2f_percentage"]]
        mean = df.groupby(['planning_bias', 'ip_style']).mean()
        std = df.groupby(['planning_bias', 'ip_style']).std()
        return mean, std

    def _import_data(self):
        return import_data_as_dataframe(self.data_directory)

