"""This script contains the definition of a node that can be used for testing and debugging new developments."""

import rospy
from rostopic import ROSTopicHz
from gazebo_msgs.msg import ContactsState
from rospy import get_published_topics
from training_q_learning.utils import write_data_to_csv
import rospkg
import os
import time


logging_hz = 3 #s
time.sleep(60)


class LogTopicFreqs():
    def __init__(self):
        #Get list of topic registered as published
        self.topic_list = [item[0] for item in get_published_topics()]

        #Create list of rostopic hz listeners and associated subscribers
        self.rostopic_hz_list = list()            
        subscribers = list()
        for i in range(len(self.topic_list)):
            self.rostopic_hz_list.append(ROSTopicHz(1500))
            subscribers.append(rospy.Subscriber(self.topic_list[i], rospy.AnyMsg, self.rostopic_hz_list[i].callback_hz, callback_args=self.topic_list[i]))
        print("Init done...")
        return
    
    def create_log_dir_path(self,parent_folder_path:str,tb_log_name:str):
        """This is a variation of the create_log_dir_path_function in the utils script. Instead of returning the path to the next folder, the latest folder's path is returned."""
        
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
        
        #Compose path to logging directory
        log_dir_path = os.path.join(parent_folder_path,tb_log_name+"_"+str(subfolder_index) )
        print("Data logging directory: ",log_dir_path)
        return log_dir_path

    def init_logging(self):
        rospack = rospkg.RosPack()
        package_name = "training_q_learning"
        self.file_path = os.path.join(self.create_log_dir_path(os.path.join(rospack.get_path(package_name), 'training_results'),package_name),"logged_topic_freqs.csv") #Create the path to a folder in which the current model as well as its checkpoints will be stored
        row = list()
        row.append("topic")
        row.append("time")
        row.append("avg_rate")
        row.append("min_delta")
        row.append("max_delta")
        row.append("std_dev")
        row.append("window")
        write_data_to_csv(self.file_path,row)
        return

    def log_stats(self):
        for i in range(len(self.topic_list)):
            row = list()
            row.append(self.topic_list[i])
            row.append(rospy.Time.now())
            tmp = self.rostopic_hz_list[i].get_hz(self.topic_list[i])
            if tmp:
                row.append(tmp[0])
                row.append(tmp[1])
                row.append(tmp[2])
                row.append(tmp[3])
                row.append(tmp[4])
                write_data_to_csv(self.file_path,row)
        return

if __name__ == '__main__':
    #Init nodes and subscribers
    rospy.init_node("log_topic_freqs")
    rate = rospy.Rate(logging_hz)
    log_node = LogTopicFreqs()
    log_node.init_logging()
    while not rospy.is_shutdown():
        log_node.log_stats()
        rate.sleep()


    
    














