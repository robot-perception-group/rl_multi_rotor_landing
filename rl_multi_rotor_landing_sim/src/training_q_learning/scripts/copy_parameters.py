'''
This script  copies all the files that are necessary to determine the settings that were used for a round of training
to the folder that has been automatically created when the logging system was set up by the Q learning node.
'''


import shutil
import os
import time
from training_q_learning.utils import create_log_dir_path
import rospkg
        


package_name = "training_q_learning"
tb_log_name = package_name
rospack = rospkg.RosPack()
outdir = os.path.join(rospack.get_path(package_name), 'training_results') #Determine path to folder to save different model and checkpoints in
log_dir_path = create_log_dir_path(outdir,tb_log_name) #Create the path to a folder in which the current model as well as its checkpoints will be stored



#Wait some time to allow logging process to start
time.sleep(30)
#Copy parameter file
#Path(log_dir_path).mkdir(parents = True,exist_ok = True)

#The path should be there, it should be created by the start_training.py script
shutil.copy2(os.path.join(rospack.get_path(package_name),'src',package_name,'parameters.py'),os.path.join(log_dir_path,'parameters.py.save'))
print("==========================================")
print("Copied file:",os.path.join(rospack.get_path(package_name),'src',package_name,'parameters.py'),'to ',os.path.join(log_dir_path,'parameters.py.save'))
print("==========================================")
shutil.copytree(os.path.join(rospack.get_path(package_name),'config'),os.path.join(log_dir_path,'config'))
print("Copied folder:",os.path.join(rospack.get_path(package_name),'config'),'to ',os.path.join(log_dir_path,'config'))
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
