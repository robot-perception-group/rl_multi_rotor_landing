"""
This script automizes the execution of the evaluation of an agent for all specified experiments. 
"""
import os
import sys
import fileinput
import shutil
import time


## Input
ros_port = sys.argv[1]
gaz_port = sys.argv[2]
sim_id = sys.argv[3]
path_to_exp = sys.argv[4]



## Calc
#Check it ROS_PROJECT_ROOT variable is set 
if not "ROS_PROJECT_ROOT" in os.environ:
    raise ValueError("ROS_PROJECT_ROOT not set. Source the file 'other_files/setup.bash'!")

for exp_id in ['vmp_0_4','vmp_0_8','vmp_1_2','vmp_1_6']:
    cmd = "python3 "+os.path.join(os.getenv("ROS_PROJECT_ROOT"),"other_files","execute_agent_evaluation_for_one_exp.py")+" "+str(ros_port)+" "+str(gaz_port)+" "+exp_id+" "+sim_id+" "+path_to_exp
    print("Running evaluation of experiment",exp_id,"for",sim_id)
    os.system(cmd)



