"""
Thoughts: The order in which the agent sees the experiences matters. 
"""
import numpy as np
import os
import pandas as pd
from copy import deepcopy
from os import listdir
from os.path import isfile, join

training_id = 'training_q_learning_5'
experiment_id = 'vmp_1_6_rmp_2_fag_22_91'
experiment_path = '/home/pgoldschmid/Desktop/experiments'
sims = ['sim_1','sim_2','sim_3','sim_4','sim_5','sim_6']


#Create list with file_paths
file_paths = dict()
for sim in sims:
    dir_path = os.path.join(experiment_path,experiment_id,sim,"training_results",training_id,"")
    files = [f for f in listdir(dir_path) if isfile(join(dir_path, f))]
    init_file_name = [s for s in files if "init_data.csv" in s][0]
    file_paths[sim] = os.path.join(dir_path, init_file_name)

#Create data dict containing the logged data as pd data frames
datas = dict()
for sim in sims:
    df = pd.read_csv(file_paths[sim],skiprows = 1,delimiter = ',') #Skip first episode which is there due to reset during training init
    df.columns = range(df.shape[1]) #Replace column names with integer values
    datas[sim] = deepcopy(df.to_numpy())

#Analyze data
for sim in sims:
    #Compute relative distance and relative velocity between mp and drone
    data = datas[sim]
    





