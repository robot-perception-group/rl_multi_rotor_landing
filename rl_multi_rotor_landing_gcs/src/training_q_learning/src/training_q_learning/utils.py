import rospy
import rosgraph
import rospkg
import os
import csv
from itertools import chain
import numpy as np
import time


def get_publisher(topic_path, msg_type, **kwargs):
    ''' Function gets a publisher and returns if the publisher is not up. Modified version of https://answers.ros.org/question/9665/test-for-when-a-rospy-publisher-become-available/?answer=367990#post-id-367990.'''

    kwargs_dict = dict(kwargs)

    #Check if port variables are set. If yes, set up the environment accordingly.
    if "ros_ip" in kwargs and "ros_port" in kwargs and "gaz_port" in kwargs_dict:
        ros_ip = kwargs_dict.get("ros_ip")
        ros_port = kwargs_dict.get("ros_port")
        gaz_port = kwargs_dict.get("gaz_port")
        print("get_publisher:",topic_path,"at ros_port = ",ros_port,",gaz_port = ",gaz_port,",ros_ip = ",ros_ip)
        host_addr = "http://" + str(ros_ip) + ":"
      #  os.environ["ROS_MASTER_URI"] = host_addr + str(ros_port) + "/"
        os.environ["GAZEBO_MASTER_URI"] = host_addr + str(gaz_port) + "/"
        kwargs_dict.pop("ros_ip")
        kwargs_dict.pop("ros_port")
        kwargs_dict.pop("gaz_port")

    pub = rospy.Publisher(topic_path, msg_type, **kwargs_dict)
    num_subs = len(_get_subscribers(topic_path))
    for i in range(10):
        num_cons = pub.get_num_connections()
        if num_cons == num_subs:
            return pub
        time.sleep(0.1)
    raise RuntimeError("Failed to get publisher ",topic_path)

def _get_subscribers(topic_path):
    '''Function gets the list of subscribers to a topic. Source: https://answers.ros.org/question/9665/test-for-when-a-rospy-publisher-become-available/?answer=367990#post-id-367990. '''
    ros_master = rosgraph.Master('/rostopic')
    topic_path = rosgraph.names.script_resolve_name('rostopic', topic_path)
    state = ros_master.getSystemState()
    subs = []
    for sub in state[1]:
        if sub[0] == topic_path:
            subs.extend(sub[1])
    return subs

def load_csv_values_as_nparray(file_path:str):
    """Function loads csv values as a numpy array."""
    with open(file_path, newline='') as f: #Read csv data and overwrite csv_date with csv values if there are any
        csv_reader = csv.reader(f)
        csv_data = list(csv_reader)
        csv_strings_list = list(chain.from_iterable(csv_data)) #unnest list
        np_values = np.array([float(i) for i in csv_strings_list]) #convert to float and numpy array
    return np_values

def load_csv_values_as_nparray_for_state(state_name:str,package_name:str):
    """Function loads the normalized limits values specified for different refinement / curriculum steps from a csv file and returns them as a numpy array."""
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path(package_name) 
    file_path = os.path.join(pkg_path,'config','discretization_files','abs_discretization_steps_'+state_name+'.csv')
    with open(file_path, newline='') as f: #Read csv data and overwrite csv_date with csv values if there are any
        csv_reader = csv.reader(f)
        csv_data = list(csv_reader)
        csv_strings_list = list(chain.from_iterable(csv_data)) #unnest list
        np_values = np.array([float(i) for i in csv_strings_list]) #convert to float and numpy array
    return np_values


def create_log_dir_path(parent_folder_path,tb_log_name):
    """Function determines a unique path where the training data should be stored within a parent folder. If a training with the same name but different training id exists, it returns a path to subsequent folder. """
    print("-----------------------------------------------")
    print("tb_log_name = ",tb_log_name)
    print("-----------------------------------------------")
    subfolder_index = int(0)
    for i in os.listdir(parent_folder_path):
        #Create absolute paths of all elements in parent_folder
        sub_path = os.path.join(parent_folder_path,i)
        #Iterate through all the sub element paths. If one is a directory that contains the string tb_log_name, execute the if clause
        if os.path.isdir(sub_path) and tb_log_name in i:
            #Get name of subfolder 
            subfolder_name = os.path.basename(os.path.normpath(sub_path))
            #get last segment of string after splitting at underscore and convert to int. This is the index number created by the training algorithm
            index = int(subfolder_name.split("_")[-1])
            if index > subfolder_index:
                subfolder_index = index
    log_dir_path = os.path.join(parent_folder_path,tb_log_name+"_"+str(subfolder_index + 1) )
    print("-----------------------------------------------")
    print("log_dir_path = ",log_dir_path)
    print("-----------------------------------------------")
    return log_dir_path


def write_data_to_csv(file_path,row_list):
    with open(file_path,'a+') as f:
            writer = csv.writer(f)
            writer.writerow(row_list)
            print("added csv entry ",row_list,"to",file_path)
    return