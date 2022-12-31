"""
This scrips extracts topics from rosbag files that are required for further analyis.
"""

import rosbag
import numpy as np
import os
import csv
from os.path import expanduser
from pathlib import Path

## Input
#Parameters
bag_name = 'stmoving_good_parameters_fast_20220824_205900.bag'
bag_file_path = '/home/pgoldschmid/Desktop/experiments/real_world/experimente_20220824'
file_base_path = 'rosbag_converts'
skipped_entries_at_start_of_episode = 5


## Calc
id_name = bag_name[:-4]
list_of_msg = []
rel_p_x_list = []
rel_p_y_list = []
rel_p_z_list = []
rel_v_x_list = []
rel_v_y_list = []
rel_v_z_list = []
t_rel_list = []
t_reset_list = []

#Read rosbag file and extract rel position and rel vel data
print("Start extracting observations...")
bag = rosbag.Bag(os.path.join(bag_file_path,bag_name))
for topic, msg, t in bag.read_messages(topics=['/copter/vicon_observation_interface/observations']):
    rel_p_x_list.append(msg.rel_p_x)
    rel_p_y_list.append(msg.rel_p_y)
    rel_p_z_list.append(msg.rel_p_z)
    rel_v_x_list.append(msg.rel_v_x)
    rel_v_y_list.append(msg.rel_v_y)
    rel_v_z_list.append(msg.rel_v_z)
    t_rel_list.append(t.to_sec())

#Make dir and initialize file
Path(os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Total')).mkdir(parents=True, exist_ok=True)
file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Total',id_name+"_observations"+'.dat')
open(file_path,'w+').close()

#Fill with data
row_list = []
for j in range(skipped_entries_at_start_of_episode,len(t_rel_list)):
    row_list = [t_rel_list[j],rel_p_x_list[j],rel_p_y_list[j],rel_p_z_list[j],rel_v_x_list[j],rel_v_y_list[j],rel_v_z_list[j]]
    with open(file_path,'a+') as f:
        writer = csv.writer(f)
        writer.writerow(row_list)
print("Done extracting observations...")

#Read rosbag file and extract position and velocity data from state estimate
print("Start extracting state estimate...")
p_x_list =[]
p_y_list =[]
p_z_list =[]
v_x_list =[]
v_y_list =[]
v_z_list =[]
t_list =[]
bag = rosbag.Bag(os.path.join(bag_file_path,bag_name))
for topic, msg, t in bag.read_messages(topics=['/fc0/pose']):
    p_x_list.append(msg.position.x)
    p_y_list.append(msg.position.y)
    p_z_list.append(msg.position.z)
    v_x_list.append(msg.velocity.x)
    v_y_list.append(msg.velocity.y)
    v_z_list.append(msg.velocity.z)
    t_list.append(t.to_sec())

#Make dir and initialize file
Path(os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4])).mkdir(parents=True, exist_ok=True)
file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Total',id_name+"_state_estimate_pos_vel"+'.dat')
open(file_path,'w+').close()

#Fill with data
row_list = []
for j in range(skipped_entries_at_start_of_episode,len(t_list)):
    row_list = [t_list[j],p_x_list[j],p_y_list[j],p_z_list[j],v_x_list[j],v_y_list[j],v_z_list[j]]
    with open(file_path,'a+') as f:
        writer = csv.writer(f)
        writer.writerow(row_list)
print("Done extracting state estimate...")

#Read rosbag file and extract position of moving platform
print("Start extracting moving platform pose in ned...")
p_x_list =[]
p_y_list =[]
p_z_list =[]
t_list =[]
bag = rosbag.Bag(os.path.join(bag_file_path,bag_name))
for topic, msg, t in bag.read_messages(topics=['/vicon/moving_platform/pose']):
    p_x_list.append(msg.pose.position.x)
    p_y_list.append(msg.pose.position.y)
    p_z_list.append(msg.pose.position.z)
    t_list.append(t.to_sec())

#Make dir and initialize file
Path(os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4])).mkdir(parents=True, exist_ok=True)
file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Total',id_name+"_mp_pose"+'.dat')
row_list = []
open(file_path,'w+').close()

#Fill with data
row_list = []
for j in range(skipped_entries_at_start_of_episode,len(t_list)):
    row_list = [t_list[j],p_x_list[j],p_y_list[j],p_z_list[j]]
    with open(file_path,'a+') as f:
        writer = csv.writer(f)
        writer.writerow(row_list)
print("Done extracting moving platform pose in ned...")

#Read rosbag file and extract velocity of moving platform
print("Start extracting moving platform velocity in ned...")
v_x_list =[]
v_y_list =[]
v_z_list =[]
t_list =[]
bag = rosbag.Bag(os.path.join(bag_file_path,bag_name))
for topic, msg, t in bag.read_messages(topics=['/vicon/moving_platform/twist']):
    v_x_list.append(msg.twist.linear.x)
    v_y_list.append(msg.twist.linear.y)
    v_z_list.append(msg.twist.linear.z)
    t_list.append(t.to_sec())

#Make dir and initialize file
Path(os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4])).mkdir(parents=True, exist_ok=True)
file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Total',id_name+"_mp_twist"+'.dat')
row_list = []
open(file_path,'w+').close()

#Fill with data
row_list = []
for j in range(skipped_entries_at_start_of_episode,len(t_list)):
    row_list = [t_list[j],v_x_list[j],v_y_list[j],v_z_list[j]]
    with open(file_path,'a+') as f:
        writer = csv.writer(f)
        writer.writerow(row_list)
print("Done extracting moving platform velocity in ned...")

print("Start extracting ROSControlled status...")
ROSControlled_list = []
t_list =[]
bag = rosbag.Bag(os.path.join(bag_file_path,bag_name))
for topic, msg, t in bag.read_messages(topics=['/fc0/TransmitterInfo']):
    ROSControlled_list.append(msg.ROSControlled)
    t_list.append(t.to_sec())

#Make dir and initialize file
Path(os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4])).mkdir(parents=True, exist_ok=True)
file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Total',id_name+"_ROSControlled"+'.dat')
row_list = []
open(file_path,'w+').close()

#Fill with data
row_list = []
for j in range(skipped_entries_at_start_of_episode,len(t_list)):
    row_list = [t_list[j],ROSControlled_list[j]]
    with open(file_path,'a+') as f:
        writer = csv.writer(f)
        writer.writerow(row_list)
print("Done extracting ROSControlled status...")


