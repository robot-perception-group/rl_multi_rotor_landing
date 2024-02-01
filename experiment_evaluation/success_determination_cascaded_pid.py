"""
This script computes the success rate for the results obtained in several simulation and determines 
elementary statistical values. 
"""

import numpy as np
import os
import pandas as pd
from copy import deepcopy

#Analysis params
mp_edge_length_x = 1
mp_edge_length_y = 1
number_of_episodes = 150


# experiment_path = '/home/pgoldschmid/Desktop/exp_revision/cascaded_pid/vmp_0_4/vmpx_0_4_vmpy_0_2_cascaded_pid_noise.csv'
experiment_path = '/home/pgoldschmid/Desktop/exp_revision/ablation_study/vmp_1_6/all_states/vmpx_1_6_vmpy_0_8_all_states.csv'
#Create data dict containing the logged data as pd data frames
datas = dict()
data_lengths = []

df = pd.read_csv(experiment_path,skiprows = 1,delimiter = ',') #Skip first landing trial which might not be complete
df.columns = range(df.shape[1]) #Replace column names with integer values
data = df.to_numpy()
data_lengths.append(len(data))

#Check if the specified number of episodes matches the number of available data entries
if not all(x >= number_of_episodes for x in data_lengths):
    print("\033[91mNot enough data to display results for",number_of_episodes,"epsiodes. Number of recorded episodes:",min(data_lengths))
    number_of_episodes = min(data_lengths)
    print("NOTE: Showing results for",number_of_episodes,"epsiodes.")

#Determine for the data set the success rate
num_success = 0
for j in range(number_of_episodes):
    if abs(data[j,15]-data[j,21]) <= mp_edge_length_x/2 and abs(data[j,16]-data[j,22]) <= mp_edge_length_y/2:
        num_success += 1
    
# print(nums_success)
success = np.rint(num_success/number_of_episodes*100)
print("Experiment:",experiment_path)
print("Showing results for",number_of_episodes,"/",min(data_lengths),"landing trials")
print("Success rate [%]: ",success)




#15 mp x pos
#21 drone x pos


