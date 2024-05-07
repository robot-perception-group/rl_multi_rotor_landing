"""
This script computes the success rate for the results obtained in several simulation and determines 
elementary statistical values. 
"""

import numpy as np
import os
import pandas as pd
from copy import deepcopy

#Analysis params
mp_edge_length_x = 0.5
mp_edge_length_y = 0.5
number_of_episodes = 150

#Specify experiments
case_id = 'vmp_0_4_real'
exp_id = 'vmpexp_x_0_vmpexp_y_0_rpm_noise'


experiment_path = '/home/pgoldschmid/Downloads/exp_paper (2)/exp_paper/simulation_data/RL'
sims = ['sim_1','sim_2','sim_3','sim_4']
sims = ['sim_3']


#Create list with file_paths
file_paths = dict()
for sim in sims:
    file_paths[sim] = os.path.join(experiment_path,case_id,"test_results","test_data_"+case_id+"_"+sim+"_"+exp_id)+".csv"

#Create data dict containing the logged data as pd data frames
datas = dict()
data_lengths = []
for sim in sims:
    df = pd.read_csv(file_paths[sim],skiprows = 20,delimiter = ',') #Skip first landing trial which might not be complete
    df.columns = range(df.shape[1]) #Replace column names with integer values
    dfnp = df.to_numpy()
    datas[sim] = deepcopy(dfnp)
    data_lengths.append(len(dfnp))

#Check if the specified number of episodes matches the number of available data entries
if not all(x >= number_of_episodes for x in data_lengths):
    print("\033[91mNot enough data to display results for",number_of_episodes,"epsiodes. Number of recorded episodes:",min(data_lengths))
    number_of_episodes = min(data_lengths)
    print("NOTE: Showing results for",number_of_episodes,"epsiodes.")

#Determine for each data set the success rate
nums_success = dict()
for sim in sims:
    data = deepcopy(datas[sim])
    num_success = 0
    for j in range(number_of_episodes):
        if abs(data[j,15]-data[j,21]) <= mp_edge_length_x/2 and abs(data[j,16]-data[j,22]) <= mp_edge_length_y/2:
            num_success += 1
        else:
            print("sim",sim,"ep.",j)
            pass
            # print("Ep.",j,"Fail:",sim,abs(data[j,15]-data[j,21]),abs(data[j,16]-data[j,22]))
    nums_success[sim] = num_success
    
# print(nums_success)
success_array = np.rint(np.array(list(nums_success.values()))/number_of_episodes*100)
print("Case:",case_id)
print("Experiment:",exp_id)
print("Number of available episodes:",min(data_lengths))
print("All success rates: ",success_array)
print("Mean Success Rate: ",np.rint(np.mean(success_array)))
print("Std Success Rate: ",np.rint(np.std(success_array)))
print("Max Success Rate: ",np.rint(np.max(success_array)))
print("Min Success Rate: ",np.rint(np.min(success_array)))
print(np.rint(np.mean(success_array)),'+-',np.rint(np.std(success_array)),np.rint(np.min(success_array)),'(',np.argmin(success_array)+1,')',np.rint(np.max(success_array)),'(',np.argmax(success_array)+1,')')



#15 mp x pos
#21 drone x pos


