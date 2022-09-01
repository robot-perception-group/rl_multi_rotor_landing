import rosbag
import numpy as np
import os
import csv
from os.path import expanduser


import matplotlib as mpl
import matplotlib.pyplot as plt



#Parameters
bag_file_path = '/home/pgoldschmid/Desktop/experiments/descend_trajectories/vmp_1_2_rmp_2_fag_17_18_rectiliar_periodic_straight_vmp_0_4_rmp_2_yaw_0.bag'
file_base_path = 'relative_descend_trajectories'
id_name = 'yaw_0'
skipped_entries_at_start_of_episode = 7

list_of_msg = []
rel_p_x_list = []
rel_p_y_list = []
rel_p_z_list = []
t_rel_list = []
t_reset_list = []

#Read rosbag file and extract rel position data
bag = rosbag.Bag(bag_file_path)
for topic, msg, t in bag.read_messages(topics=['/hummingbird/training_observation_interface/observations']):
    rel_p_x_list.append(getattr(msg,"rel_p_x"))
    rel_p_y_list.append(getattr(msg,"rel_p_y"))
    rel_p_z_list.append(getattr(msg,"rel_p_z"))
    header = getattr(msg,"header")
    t_rel_list.append(header.stamp.to_sec())

print("t_reset_list =",t_reset_list)

for topic, msg, t in bag.read_messages(topics=['/hummingbird/training/reset_simulation']):
    t_reset_list.append(t.to_sec())
bag.close()
print(t_reset_list)
#Convert to np arrays
rel_p_x = np.array(rel_p_x_list)
rel_p_y = np.array(rel_p_y_list)
rel_p_z = np.array(rel_p_z_list)
t_rel = np.array(t_rel_list)
t_reset = np.array(t_reset_list)


alt_diff = np.diff(rel_p_z)
idx_reset = np.where(np.abs(alt_diff)>1 )
# print(idx_reset)

#Iterate through each episode
idx_starts = []
idx_stops = []

for i in range(len(t_reset)-1):
    idx_starts.append(np.argmax(t_rel > t_reset[i]))
    idx_stops.append(np.argmax(t_rel > t_reset[i+1]-1))
    
print(idx_starts)
print(idx_stops)

for i in range(1,len(idx_starts)-1):
    #Iterate through each data point
    
    #Create empty file to be filled with data
    file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,id_name+"_"+str(i)+'.dat')
    print(file_path)
    row_list = []
    open(file_path,'w+').close()

    #Fill with data
    row_list = []
    print("Index range:",idx_starts[i]+skipped_entries_at_start_of_episode,idx_stops[i])
    for j in range(idx_starts[i]+skipped_entries_at_start_of_episode,idx_stops[i]):
        row_list = [-rel_p_x[j],-rel_p_y[j],-rel_p_z[j]]
        with open(file_path,'a+') as f:
            writer = csv.writer(f)
            writer.writerow(row_list)
