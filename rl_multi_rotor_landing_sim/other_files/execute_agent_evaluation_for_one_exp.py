"""
This script automizes the execution of the evaluation of an agent. 
"""
import os
import sys
import fileinput
import shutil
import time


## Input
ros_port = sys.argv[1]
gaz_port = sys.argv[2]
exp_id = sys.argv[3]
sim_id = sys.argv[4]
path_to_exp = sys.argv[5]

eval_time = 4500       #[s]
kill_screen_break_time = 5 #[s]
test_model_init_time = 15 #[s]

## Calc
#Functions
def replace_text_in_file(file_path:str,pattern:str,replace_text:str):
    """This functions opens a file, searches for a pattern, replaces it with the replacement text and saves the modified file."""
    if not os.path.isfile(file_path): raise FileExistsError(file_path+"could not be found")
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
    if not os.path.isfile(file_path): raise FileExistsError(file_path+"could not be found")
    for line in fileinput.input(file_path,inplace=True):
        if pattern in line:
            line = replace_line+'\n'
        sys.stdout.write(line)    
    return

def get_last_final_result_path():
    """Function returns the absolute path to the latest final result that can be used as a starting point for the subsequent curriculum step."""
    #Path to the training_results directory
    training_results_path = os.path.join(path_to_exp,exp_id,sim_id,"training_results")

    #Get the list of folders in training_results in alphabetically ascending order
    subfolders = sorted([ f.path for f in os.scandir(training_results_path) if f.is_dir() ])
    
    #Get the latest_training_results_path
    latest_training_results_path = subfolders[-1]
    files = [f for f in os.listdir(latest_training_results_path) if os.path.isfile(os.path.join(latest_training_results_path, f)) and "FINAL_Q_table.npy" in f]
    final_result_path = os.path.join(latest_training_results_path,files[0][:-12])
    return final_result_path

def get_last_parameters_file(exp_id:str,sim_id:str):
    """Function returns the path to the last parameters.py file of a sequential training curriculum"""
    #Path to the training_results directory
    training_results_path = os.path.join(path_to_exp,exp_id,sim_id,"training_results")

    #Get the list of folders in training_results in alphabetically ascending order
    subfolders = sorted([ f.path for f in os.scandir(training_results_path) if f.is_dir() ])
    
    #Get the latest_training_results_path
    latest_training_results_path = subfolders[-1]
    return os.path.join(latest_training_results_path,"parameters.py.save")


#Check if ROS_PROJECT_ROOT variable is set 
if not "ROS_PROJECT_ROOT" in os.environ:
    raise ValueError("ROS_PROJECT_ROOT not set. Source the file 'other_files/setup.bash'!")


#Copy the latest parameter file to the appropriate directory
parameters_path = os.path.join(os.getenv("ROS_PROJECT_ROOT"),"src","training_q_learning","src","training_q_learning","parameters.py")
src_path = get_last_parameters_file(exp_id,sim_id)
shutil.copyfile(src_path,parameters_path)

#Perform update of parameters on the parameters file
last_final_results_path = get_last_final_result_path()
replace_line_in_textfile_that_contains(parameters_path,"self.load_data_from","        self.load_data_from = '"+last_final_results_path+"'")
#Update yaw value
replace_text_in_file(parameters_path,'"yaw":0','"yaw":np.pi/4')
#Update time
replace_text_in_file(parameters_path,'self.t_max: float = 20 #[s]','self.t_max: float = 23')
#Update init distribution
replace_text_in_file(parameters_path,"self.init_distribution: str = 'normal'","self.init_distribution: str = 'uniform'")
#Update init altitude
replace_text_in_file(parameters_path,"self.init_altitude: float = 4","self.init_altitude: float = 2.5")
#Update terminal failure for reaching goal state
replace_text_in_file(parameters_path,'"success" : True','"success" : False')
replace_text_in_file(parameters_path,'"max_num_timesteps" : True','"max_num_timesteps" : False')

#Perform the evaluation for the different scenarios
analysis_path = os.path.join(os.getenv("ROS_PROJECT_ROOT"),"src","training_q_learning","scripts","analysis_node_2D.py")

#Rectilinear periodic movement with different velocities
for vmp_x in [0,0.4,0.8,1.2,1.6]:
    vmp_y = 0
    #Lateral velocity
    #Kill the terminal screen containing the publisher to the topic
    cmd_vmp_x_kill ='screen -S "'+sim_id+'_vmp_x_setpoint" -p 0 -X quit'
    os.system(cmd_vmp_x_kill)
    cmd_vmp_y_kill ='screen -S "'+sim_id+'_vmp_y_setpoint" -p 0 -X quit'
    os.system(cmd_vmp_y_kill)
    time.sleep(kill_screen_break_time)
    cmd_test_model_kill ='screen -S "'+sim_id+'_test_model" -p 0 -X quit'
    os.system(cmd_test_model_kill)
    cmd_analysis_kill ='screen -S "'+sim_id+'_analysis" -p 0 -X quit'
    os.system(cmd_analysis_kill)
    time.sleep(kill_screen_break_time)
    
    #Publish
    cmd_vmp_x_publish ='screen -d -m -S "'+sim_id+'_vmp_x_setpoint" bash -i -c "'+os.getenv("ROS_PROJECT_ROOT")+'/other_files/publish_vmp_x.sh '+str(vmp_x)+' '+str(ros_port)+' '+str(gaz_port)+'" &'
    os.system(cmd_vmp_x_publish) 
    cmd_vmp_y_publish ='screen -d -m -S "'+sim_id+'_vmp_y_setpoint" bash -i -c "'+os.getenv("ROS_PROJECT_ROOT")+'/other_files/publish_vmp_y.sh '+str(vmp_y)+' '+str(ros_port)+' '+str(gaz_port)+'" &'
    os.system(cmd_vmp_y_publish) 
    
    #Perform updates on the analysis file
    test_results_path = os.path.join(path_to_exp,sim_id,"test_results")
    file_name_components = [exp_id,sim_id,"vmpexp_x",str(vmp_x),"vmpexp_y","0","rmp"]
    test_results_file_name = "_".join(file_name_components).replace(".","_")
    file_path_csv = os.path.join(path_to_exp,exp_id,"test_results","test_data_"+test_results_file_name+".csv")
    replace_line_in_textfile_that_contains(analysis_path,"id = ","id = '"+test_results_file_name+"'")
    replace_line_in_textfile_that_contains(analysis_path,"file_path_csv = ","file_path_csv = '"+file_path_csv+"'")

    #Launch test model script
    print("\tRectilinear movement with vmp_x =",vmp_x,"and vmp_y =",vmp_y)
    cmd_launch_test_model ='screen -d -m -S "'+sim_id+'_test_model" bash -i -c "'+os.path.join(os.getenv("ROS_PROJECT_ROOT"),"src","training_q_learning","launch","launch_test_model_2D.sh")+" hummingbird"+" "+str(ros_port)+" "+str(gaz_port)+'" &'
    os.system(cmd_launch_test_model)
    time.sleep(test_model_init_time)
    #Launch analysis script
    cmd_analysis_model ='screen -d -m -S "'+sim_id+'_analysis" bash -i -c "'+os.path.join(os.getenv("ROS_PROJECT_ROOT"),"src","training_q_learning","launch","launch_analysis_node_2D.sh")+" hummingbird"+" "+str(ros_port)+" "+str(gaz_port)+'" &'
    os.system(cmd_analysis_model)
    time.sleep(eval_time)

#Eight shape movement
#Extract the platform velocity with which the training was executed. 
#IMPORTANT: Formatting the of the exp_id needs to be "vmp_A_B..." where A B denote the components of the velocity vmp = A.B in m/s
vmp = float(exp_id[4:7].replace("_","."))
#Kill the terminal screen containing the publisher to the topic
cmd_vmp_x_kill ='screen -S "'+sim_id+'_vmp_x_setpoint" -p 0 -X quit'
os.system(cmd_vmp_x_kill)
cmd_vmp_y_kill ='screen -S "'+sim_id+'_vmp_y_setpoint" -p 0 -X quit'
os.system(cmd_vmp_y_kill)
cmd_test_model_kill ='screen -S "'+sim_id+'_test_model" -p 0 -X quit'
os.system(cmd_test_model_kill)
cmd_analysis_kill ='screen -S "'+sim_id+'_analysis" -p 0 -X quit'
os.system(cmd_analysis_kill)
time.sleep(kill_screen_break_time)

#Publish
cmd_vmp_x_publish ='screen -d -m -S "'+sim_id+'_vmp_x_setpoint" bash -i -c "'+os.getenv("ROS_PROJECT_ROOT")+'/other_files/publish_vmp_x.sh '+str(vmp)+' '+str(ros_port)+' '+str(gaz_port)+'" &'
os.system(cmd_vmp_x_publish) 
cmd_vmp_y_publish ='screen -d -m -S "'+sim_id+'_vmp_y_setpoint" bash -i -c "'+os.getenv("ROS_PROJECT_ROOT")+'/other_files/publish_vmp_y.sh '+str(vmp/2)+' '+str(ros_port)+' '+str(gaz_port)+'" &'
os.system(cmd_vmp_y_publish) 

#Perform updates on the analysis file
test_results_path = os.path.join(path_to_exp,sim_id,"test_results")
file_name_components = [exp_id,sim_id,"vmpexp_x",str(vmp),"vmpexp_y",str(vmp/2),"eight_shape"]
test_results_file_name = "_".join(file_name_components).replace(".","_")
file_path_csv = os.path.join(path_to_exp,exp_id,"test_results","test_data_"+test_results_file_name+".csv")
replace_line_in_textfile_that_contains(analysis_path,"id = ","id = '"+test_results_file_name+"'")
replace_line_in_textfile_that_contains(analysis_path,"file_path_csv = ","file_path_csv = '"+file_path_csv+"'")

#Launch test model script
print("\tEight shape movement with vmp_x =",vmp,"and vmp_y =",vmp/2)
cmd_launch_test_model ='screen -d -m -S "'+sim_id+'_test_model" bash -i -c "'+os.path.join(os.getenv("ROS_PROJECT_ROOT"),"src","training_q_learning","launch","launch_test_model_2D.sh")+" hummingbird"+" "+str(ros_port)+" "+str(gaz_port)+'" &'
os.system(cmd_launch_test_model)
time.sleep(test_model_init_time)

#Launch analysis script
cmd_analysis_model ='screen -d -m -S "'+sim_id+'_analysis" bash -i -c "'+os.path.join(os.getenv("ROS_PROJECT_ROOT"),"src","training_q_learning","launch","launch_analysis_node_2D.sh")+" hummingbird"+" "+str(ros_port)+" "+str(gaz_port)+'" &'
os.system(cmd_analysis_model)
time.sleep(eval_time)



