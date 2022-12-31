'''
This script  copies all the files that are necessary to determine the settings that were used for a round of training
to the folder that has been automatically created when the logging system was set up by the Q learning node.
'''


import shutil
import os
import time
import rospkg


def create_log_dir_path(parent_folder_path:str,tb_log_name:str):
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

#Wait some time to allow logging process to start
package_name = "training_q_learning"
tb_log_name = package_name
rospack = rospkg.RosPack()
outdir = os.path.join(rospack.get_path(package_name), 'training_results') #Determine path to folder to save different model and checkpoints in
log_dir_path = create_log_dir_path(outdir,tb_log_name) #Create the path to a folder in which the current model as well as its checkpoints will be stored
time.sleep(40)

#The path should be there, it should be created by the training.py script
shutil.copy2(os.path.join(rospack.get_path(package_name),'src',package_name,'parameters.py'),os.path.join(log_dir_path,'parameters.py.save'))
print("==========================================")
print("Copied file:",os.path.join(rospack.get_path(package_name),'src',package_name,'parameters.py'),'to ',os.path.join(log_dir_path,'parameters.py.save'))
print("==========================================")
shutil.copy2(os.path.join(rospack.get_path(package_name),'src',package_name,'landing_simulation_object.py'),os.path.join(log_dir_path,'landing_simulation_object.py.save'))
print("==========================================")
print("Copied file:",os.path.join(rospack.get_path(package_name),'src',package_name,'landing_simulation_object.py'),'to ',os.path.join(log_dir_path,'landing_simulation_object.py.save'))
print("==========================================")
shutil.copy2(os.path.join(rospack.get_path(package_name),'src',package_name,'custom_q_learning.py'),os.path.join(log_dir_path,'custom_q_learning.py.save'))
print("==========================================")
print("Copied file:",os.path.join(rospack.get_path(package_name),'src',package_name,'custom_q_learning.py'),'to ',os.path.join(log_dir_path,'custom_q_learning.py.save'))
print("==========================================")
shutil.copy2(os.path.join(rospack.get_path(package_name),'launch','landing_simulation.launch'),os.path.join(log_dir_path,'landing_simulation.launch.save'))
print("==========================================")
print("Copied file:",os.path.join(rospack.get_path(package_name),'launch','landing_simulation.launch'),'to ',os.path.join(log_dir_path,'landing_simulation.launch.save'))
print("==========================================")
