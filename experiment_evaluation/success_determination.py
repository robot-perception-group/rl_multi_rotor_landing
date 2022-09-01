import numpy as np
import os
import pandas as pd
from copy import deepcopy


#Analysis params
mp_edge_length_x = 1
mp_edge_length_y = 1
number_of_episodes = 150

#Specify experiments
# experiment_id = 'rectiliar_periodic_straight_vmp_0_0_rmp_0_5_yaw_pi_4'
# experiment_id = 'eight_shape_vmpx_0_4_rmpx_0_5_vmpy_0_2_rmpy_0_5_yaw_pi_4'
experiment_id = 'rectiliar_periodic_straight_vmp_0_0_rmp_0_5_yaw_pi_4_noise_0_25_0_25_0_1_0_1_0_05'
# experiment_id = 'yaw_pi_4_static'
# experiment_id = 'yaw_pi_4_eight_shape_vmpx_1_6_rmpx_2_vmpy_0_8_rmpy_2'
training_id = 'vmp_0_4_rmp_0_5_fag_22_91'
experiment_path = '/home/pgoldschmid/Desktop/experiments'
sims = ['sim_1','sim_2','sim_3','sim_4','sim_5','sim_6']
sims = ['sim_3']


#Create list with file_paths
file_paths = dict()
for sim in sims:
    file_paths[sim] = os.path.join(experiment_path,training_id,sim,"test_results","test_data_"+sim+"_"+training_id+"_"+experiment_id)+".csv"

#Create data dict containing the logged data as pd data frames
datas = dict()
for sim in sims:
    df = pd.read_csv(file_paths[sim],skiprows = 1,delimiter = ',') #Skip first landing trial which might not be complete
    df.columns = range(df.shape[1]) #Replace column names with integer values
    datas[sim] = deepcopy(df.to_numpy())

#Determine for each data set the success rate
nums_success = dict()
for sim in sims:
    data = deepcopy(datas[sim])
    num_success = 0

    for j in range(number_of_episodes):
        if abs(data[j,15]-data[j,21]) <= mp_edge_length_x/2 and abs(data[j,16]-data[j,22]) <= mp_edge_length_y/2:
            num_success += 1
    nums_success[sim] = num_success
    
# print(nums_success)
success_array = np.rint(np.array(list(nums_success.values()))/number_of_episodes*100)
print("Training:",training_id)
print("Experiment:",experiment_id)
print(success_array)
print("Mean Success Rate: ",np.rint(np.mean(success_array)))
print("Std Success Rate: ",np.rint(np.std(success_array)))
print("Max Success Rate: ",np.rint(np.max(success_array)))
print("Min Success Rate: ",np.rint(np.min(success_array)))
print(np.rint(np.mean(success_array)),'+-',np.rint(np.std(success_array)),np.rint(np.min(success_array)),'(',np.argmin(success_array)+1,')',np.rint(np.max(success_array)),'(',np.argmax(success_array)+1,')')



#15 mp x pos
#21 drone x pos


