import numpy as np
import os
from pathlib import Path
import csv




#Parameters
bag_name = 'stmoving_good_parameters_slow_20220824_204346.bag'
file_base_path = 'rosbag_converts'

min_fly_zone_height = 1
min_armed_len = 5 #s
min_flight_time = 5 #s


#Functions 
def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return idx

#Create list of periods in which it ROS mode is activated
id_name = bag_name[:-4]
#Iterate through all ROS Controlled msg
Path(os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],"Flights")).mkdir(parents=True, exist_ok=True)
file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Total',id_name+"_ROSControlled"+'.dat')
ROSControlled_data = np.genfromtxt(file_path,delimiter=',')
print(file_path)
#Iterate through entries and determine periods beginning and end
period_active = False
current_start = 0.0
current_stop = 0.0
period_list = []
for i in range(len(ROSControlled_data)):
    #If ros mode is not activated and also no period begin found
    if int(ROSControlled_data[i,1]) == 1 and not period_active:
        #store the new beginning
        current_start = ROSControlled_data[i,0]
        period_active = True
    #If a period was stared and now ROS mode is not on anymore
    elif int(ROSControlled_data[i,1]) == 0 and period_active:
        #Store the end variable
        current_stop = ROSControlled_data[i,0]
        if current_stop - current_start > min_armed_len:
            period_list.append([current_start,current_stop])
        period_active = False
        current_start = 0.0
        current_stop = 0.0
print("Detected "+str(len(period_list))+" periods with length > " + str(min_armed_len)+"s in which ROS mode was activated.")

#Analyze when  the state estimate p_z is below min boundary of fly box for the first time.
file_path_se = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Total',id_name+"_state_estimate_pos_vel"+'.dat')
se_p_z_data = np.genfromtxt(file_path_se,delimiter=',')
for i in range(len(period_list)):
    for k in range(len(se_p_z_data)):
        if se_p_z_data[k][2] < min_fly_zone_height and se_p_z_data[k][0] < period_list[0][1]:
            #Overwrite period end with new end point
            period_list[i][1] = se_p_z_data[k][0]
            break
print("Corrected the end point based on when the fly box was left for the first time.")


#Extract flights for  mp pose
print("Start extracting flights for mp pose...")
file_path_mp_pose = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Total',id_name+"_mp_pose"+'.dat')
mp_pose_data = np.genfromtxt(file_path_mp_pose,delimiter=',')
#Iterate through periods
for i in range(len(period_list)):
    if min_flight_time > period_list[i][1]-period_list[i][0]:
        continue
    Path(os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],"Flights","flight_"+str(i))).mkdir(parents=True, exist_ok=True)

    #Create file
    file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Flights',"flight_"+str(i),id_name+"_mp_pose"+'.dat')
    row_list = []
    open(file_path,'w+').close()
    #Fill with data
    row_list = []
    
    for k in range(len(mp_pose_data)):
        if (mp_pose_data[k][0] > period_list[i][0] and mp_pose_data[k][0]< period_list[i][1]):
            row_list = mp_pose_data[k]
            row_list[0] = row_list[0] - period_list[i][0]

            with open(file_path,'a+') as f:
                writer = csv.writer(f)
                writer.writerow(row_list)
print("Done extracting flights for mp pose...")







#Extract flights for mp twist
print("Start extracting flights for mp twist...")
file_path_mp_twist = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Total',id_name+"_mp_twist"+'.dat')
mp_twist_data = np.genfromtxt(file_path_mp_twist,delimiter=',')
mp_twist_start_time = mp_twist_data[0][0]

#Iterate through periods
for i in range(len(period_list)):
    if min_flight_time > period_list[i][1]-period_list[i][0]:
        continue
    Path(os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],"Flights","flight_"+str(i))).mkdir(parents=True, exist_ok=True)
    #Create file
    file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Flights',"flight_"+str(i),id_name+"_mp_twist"+'.dat')
    row_list = []
    open(file_path,'w+').close()
    #Fill with data
    row_list = []
    k = 0
    for k in range(len(mp_twist_data)):
        if (mp_twist_data[k][0] > period_list[i][0] and mp_twist_data[k][0]< period_list[i][1]):
            row_list = mp_twist_data[k]
            row_list[0] = row_list[0] - period_list[i][0]

            with open(file_path,'a+') as f:
                writer = csv.writer(f)
                writer.writerow(row_list)
print("Done extracting flights for mp twist...")


#Extract flights for observations
print("Start extracting flights for observations...")
file_path_mp_observations = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Total',id_name+"_observations"+'.dat')
observations_data = np.genfromtxt(file_path_mp_observations,delimiter=',')
observations_start_time = observations_data[0][0]
#Iterate through periods
for i in range(len(period_list)):
    if min_flight_time > period_list[i][1]-period_list[i][0]:
        continue
    Path(os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],"Flights","flight_"+str(i))).mkdir(parents=True, exist_ok=True)
    #Create file
    file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Flights',"flight_"+str(i),id_name+"_observations"+'.dat')
    row_list = []
    open(file_path,'w+').close()
    #Fill with data
    row_list = []
    k = 0
    for k in range(len(observations_data)):
        if (observations_data[k][0] > period_list[i][0] and observations_data[k][0]< period_list[i][1]):
            row_list = observations_data[k]
            row_list[0] = row_list[0] - period_list[i][0]
            with open(file_path,'a+') as f:
                writer = csv.writer(f)
                writer.writerow(row_list)
print("Done extracting flights for observations...")

#Extract flights for state estimate
print("Start extracting flights for state estimate...")
file_path_state_estimate = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Total',id_name+"_state_estimate_pos_vel"+'.dat')
state_estimate_data = np.genfromtxt(file_path_state_estimate,delimiter=',')

#Iterate through periods
for i in range(len(period_list)):
    if min_flight_time > period_list[i][1]-period_list[i][0]:
        continue
    Path(os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],"Flights","flight_"+str(i))).mkdir(parents=True, exist_ok=True)
    #Create file
    file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Flights',"flight_"+str(i),id_name+"_state_estimate_pos_vel"+'.dat')
    row_list = []
    open(file_path,'w+').close()
    #Fill with data
    row_list = []
    k = 0
    for k in range(len(state_estimate_data)):
        if (state_estimate_data[k][0] > period_list[i][0] and state_estimate_data[k][0]< period_list[i][1]):
            row_list = state_estimate_data[k]
            row_list[0] = row_list[0] - period_list[i][0]
            with open(file_path,'a+') as f:
                writer = csv.writer(f)
                writer.writerow(row_list)
print("Done extracting flights for state estimate...")
# state_estimate_time_array = []
# for j in range(len(state_estimate_data)):
#     state_estimate_time_array.append(state_estimate_data[j][0])
# #Extract flights for  mp pose
# print("Start extracting flights for mp pose...")
# file_path_mp_pose = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Total',id_name+"_mp_pose"+'.dat')
# mp_pose_data = np.genfromtxt(file_path_mp_pose,delimiter=',')
# #Iterate through periods
# for i in range(len(period_list)):
#     if min_flight_time > period_list[i][1]-period_list[i][0]:
#         continue
#     Path(os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],"Flights","flight_"+str(i))).mkdir(parents=True, exist_ok=True)

#     #Create file
#     file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Flights',"flight_"+str(i),id_name+"_ned_pos_error"+'.dat')
#     row_list = []
#     open(file_path,'w+').close()
#     #Fill with data
#     row_list = []
#     idx_old = 0
#     for k in range(len(mp_pose_data)):
#         if (mp_pose_data[k][0] > period_list[i][0] and mp_pose_data[k][0]< period_list[i][1]):
#             print("mp time =",mp_pose_data[k][0])
#             idx = find_nearest(state_estimate_time_array,mp_pose_data[k][0])
#             if idx is not idx_old:
#                 print("new idx, k =",k)
#             print(idx)
#             row_list = [mp_pose_data[k][0],mp_pose_data[k][1]-state_estimate_data[idx][1],mp_pose_data[k][2]-state_estimate_data[idx][2],mp_pose_data[k][3]-state_estimate_data[idx][3]]
#             row_list[0] = row_list[0] - period_list[i][0]
#             with open(file_path,'a+') as f:
#                 writer = csv.writer(f)
#                 writer.writerow(row_list)
#             idx_old = idx
# print("Done extracting flights for mp pose...")








