"""This script contains the definition of a node that can be used for testing and debugging new developments."""

import rospy
from rostopic import ROSTopicHz
from gazebo_msgs.msg import ContactsState
from rospy import get_published_topics
from training_q_learning.utils import write_data_to_csv, create_log_dir_path
import rospkg
import os
import rosnode
import roslaunch 
import time



class DebugNode():
    def __init__(self):
        node = roslaunch.core.Node("training_q_learning","log_topic_freqs.py",name="log_topic_freqs_node")
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)
        print(process.is_alive())
        process.stop()

        return

    def launch_node(self):
        pass

    def kill_node(self,node_id):
        pass
        return

if __name__ == '__main__':
    #Init nodes and subscribers
    rospy.init_node("debug_node")
    debug_node = DebugNode()
    rospy.spin()



    
    














