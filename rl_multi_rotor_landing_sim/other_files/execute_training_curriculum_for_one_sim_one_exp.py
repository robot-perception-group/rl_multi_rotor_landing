"""
This script automizes the execution of the sequential curriculum. 
Requirements:
    - directory containing the training results of previous runs needs to be empty
"""
import os
import sys
import fileinput
import time
import numpy as np


## Input
n_cs = 3

## Calc
#Functions
def replace_text_in_file(file_path:str,pattern:str,replace_text:str):
    """This functions opens a file, searches for a pattern, replaces it with the replacement text and saves the modified file."""
    
    #Open file as read
    with open(file_path, 'r') as file:
        fd = file.read()

    # Find and replace the pattern string
    fd = fd.replace(pattern, replace_text)
    
    # Write the modified file
    with open(file_path, 'w') as file:
        file.write(fd)
    return

def replace_line_in_textfile_that_contains(file_path:str,pattern:str,replace_line:str):
    """
    Functions open a text file, searches through all lines and replaces the lines that contain the string 'pattern' with the string in 'replace_line'
    """
    for line in fileinput.input(file_path,inplace=True):
        if pattern in line:
            line = replace_line+'\n'
        sys.stdout.write(line)    
    return

def get_last_final_result_path():
    """Function returns the absolute path to the latest final result that can be used as a starting point for the subsequent curriculum step."""
    #Path to the training_results directory
    training_results_path = os.path.join(os.getenv("ROS_PROJECT_ROOT"),"src","training_q_learning","training_results")

    #Get the list of folders in training_results in alphabetically ascending order
    subfolders = sorted([ f.path for f in os.scandir(training_results_path) if f.is_dir() ])
    
    #Get the latest_training_results_path
    latest_training_results_path = subfolders[-1]
    files = [f for f in os.listdir(latest_training_results_path) if os.path.isfile(os.path.join(latest_training_results_path, f)) and "FINAL_Q_table.npy" in f]
    final_result_path = os.path.join(latest_training_results_path,files[0][:-12])
    return final_result_path


#Check it ROS_PROJECT_ROOT variable is set 
if not "ROS_PROJECT_ROOT" in os.environ:
    raise ValueError("ROS_PROJECT_ROOT not set. Source the file 'other_files/setup.bash'!")


#Perform initial curriculum step
cur_command = "bash "+os.path.join(os.getenv("ROS_PROJECT_ROOT"),"src","training_q_learning","launch","launch_training.sh")+" "+sys.argv[1]+" "+sys.argv[2]+" "+sys.argv[3]
os.system(cur_command)

#Perform update of parameters on the parameters file
parameters_path = os.path.join(os.getenv("ROS_PROJECT_ROOT"),"src","training_q_learning","src","training_q_learning","parameters.py")
replace_text_in_file(parameters_path,"self.number_new_curriculum_steps: int = 0","self.number_new_curriculum_steps: int = 1")
last_final_results_path = get_last_final_result_path()
replace_line_in_textfile_that_contains(parameters_path,"self.load_data_from","        self.load_data_from = '"+last_final_results_path+"'")
replace_line_in_textfile_that_contains(parameters_path,"self.exploration_rate_schedule","        self.exploration_rate_schedule: dict = {0:['lin',0,1,0,0]}")
replace_text_in_file(parameters_path,"self.exploration_initial_eps: float = 1","self.exploration_initial_eps: float = 0")

#Perform the subsequent curriculum steps
suc_frac = 0.96
for i in range(1,n_cs+1):
    suc_frac_new = np.clip(suc_frac+0.01,0,1)
    # replace_text_in_file(parameters_path,"self.successful_fraction: float = "+str(suc_frac),"self.successful_fraction: float = "+str(suc_frac_new))
    replace_text_in_file(parameters_path,"self.init_distribution: str = 'normal'","self.init_distribution: str = 'uniform'")
    suc_frac = suc_frac_new
    replace_line_in_textfile_that_contains(parameters_path,"self.curriculum_step","        self.curriculum_step: int = "+str(i))
    last_final_results_path = get_last_final_result_path()
    replace_line_in_textfile_that_contains(parameters_path,"self.load_data_from","        self.load_data_from = '"+last_final_results_path+"'")
    time.sleep(10)
    os.system(cur_command)


