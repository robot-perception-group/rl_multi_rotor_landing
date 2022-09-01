# Reinforcement Learning based Autonomous Multi-Rotor Landing on Moving Platforms

Welcome to the repository providing the code used for the paper:

"RL-based Autonomous Multi-Rotor Landing on Moving Platforms".

By Pascal Goldschmid and Aamir Ahmad

Please fine the associated flight experiment data [here](https://keeper.mpdl.mpg.de/f/949c85af353a422eb5a2/?dl=1).
If you have any questions, comments or suggestions please contact pascal.goldschmid@ifr.uni-stuttgart.de

## General
### Scope of the code
The code has been developed with Python3 for Ubuntu 20, using ROS Noetic and Gazebo 11. It provides all necessary scripts to set up the RL training environment in Gazebo. Furthermore, it contains all necessary files to apply the sequential curriculum used to train agents for different scenarios. It is possible to run several trainings in parallel in independent simulations. Furthermore, the repository contains all files necessary to deploy an agent on real hardware. 

### Structure of the repository
The repository provides three folders that contain the files to set up a ROS catkin workspace for training and evaluation in simulation ([rl_multi_rotor_landing_sim](rl_multi_rotor_landing_sim)) or for deploying an agent on real hardware. For the latter, one catkin workspace needs to be set up on a computer acting as ground control station ([rl_multi_rotor_landing_gcs](rl_multi_rotor_landing_gcs)) and the other one ([rl_multi_rotor_landing_uav](rl_multi_rotor_landing_uav)) has to run on the UAV. Dependencies are installed in the submodules folder. Code used in more than one catkin workspace is found in the src folder.

### Structure of code
The code is set up in a way that each ROS node is run in a separate virtual screen. To get a list of all virtual screens that are currently active, run
```
screen -list
```
To attach to a screen, use
```
screen -r screen_name
```
To detach from a screen, press ctrl+a and then ctrl+d.

## Requirements
### Linux packages
Install the screen package first by using the commands
```
sudo apt update
sudo apt install screen
```


### Download the code
The code includes several submodules that need to be included when downloading the code. This can be achieved with the following command
```
git clone --recurse-submodules <url to repository>
```


### Install ROS and Gazebo
You can find the installation instructions for ROS Noetic and Gazebo  [here](http://wiki.ros.org/noetic/Installation/Ubuntu).
A list of additional required ROS packages is given in [here](ros_packages_list.txt). You can install a package by using apt as follows
```
sudo apt install ros-noetic-package-name
```
### Install Python modules
A list of required additional python modules is given [here](requirements.txt). You can install it by running
```
pip3 install -r requirements.txt
```
If you are using a virtual environment or need to adapt your ```PYTHONPATH``` variable please update [this](rl_multi_rotor_landing_sim/other_files/activate_training_venv.sh) file according to your needs.

### Initialize the catkin workspaces
Make sure ROS is installed, then run
```
./init_workspaces.sh
```

## Run the code - quick start
For a quick start, you can test the training procedure in simulation.

### Set up a training environment
Prepare the catkin_workspace first by the following commands
```
cd path/to/downloaded/directory/rl_multi_rotor_landing_sim
catkin_make
source other_files/setup.bash
```
Then launch the simulation environment by running 

```
cd path/to/downloaded/directory/rl_multi_rotor_landing_sim
bash src/training_q_learning/scripts/training/launch_environment_in_virtual_screens.sh SIM_ID UAV_NAME ROS_PORT GAZEBO_PORT
```
Define the name of your environment by the parameter ```SIM_ID```. The parameter ```UAV_NAME``` can be used to select a vehicle from the [RotorS](https://github.com/ethz-asl/rotors_simulator) package. We used the UAV called "hummingbird". Make sure that the uav name that you specify here is identical to the one specified in the [parameters](rl_multi_rotor_landing_sim/src/training_q_learning/src/training_q_learning/parameters.py) file. ```ROS_PORT``` and ```GAZEBO_PORT``` are required to run several indepedent trainings in simulation at the same time. For each training, select a unique pair of ports. For ```ROS_PORT```, a working value range constitute the integers 11311 - 11316 whereas for ```GAZEBO_PORT``` it is 11351 - 11356. An example to launch the simulation is
```
cd path/to/downloaded/directory/rl_multi_rotor_landing_sim
bash src/training_q_learning/scripts/training/launch_environment_in_virtual_screens.sh test hummingbird 11311 11351
```
To run commands such as ```rostopic list``` in a different terminal window, execute the script 
```
source other_files/prepare_terminal_window.sh ROS_PORT GAZEBO_PORT
```
first, where ROS_PORT and GAZEBO_PORT need to match the one used for launching the simulation.


### Train an agent
There are several steps to be executed in the training curriculum.

 1. Run the first step of the curriculum by 
    ```
    cd path/to/downloaded/directory/rl_multi_rotor_landing_sim
    bash src/training_q_learning/scripts/training/launch_start_training_q_learning.sh UAV_NAME ROS_PORT GAZEBO_PORT
    ```
 2. After the training has finished, update the parameter ```load_training_from``` in the [parameters](rl_multi_rotor_landing_sim/src/training_q_learning/src/training_q_learning/parameters.py) file with the path to the directory where the results have been stored.  
    By default, the training results are stored [here](rl_multi_rotor_landing/src/training_q_learning/training_results).
    An example path can look like this 
    ```
    path/to/directory/of/catkin_ws/src/training_q_learning/training_results/training_q_learning_1/episode_2007_FINAL
    ```
 3. Increment the value of the parameter ```curriculum_step``` by 1 in the [parameters](rl_multi_rotor_landing_sim/src/training_q_learning/src/training_q_learning/parameters.py)  file.
   
 4. Restart the training for the next curriculum step by running 
   ```
   cd path/to/downloaded/directory/rl_multi_rotor_landing_sim
   bash src/training_q_learning/scripts/training/launch_restart_training_q_learning.sh UAV_NAME ROS_PORT GAZEBO_PORT    
   ```   
 5. With the default settings, you can repeat the steps 2 to 4 four times to achieve a fully trained agent.

### Test an agent
To test an agent in simulation, replace the respective parameters in the [parameters](rl_multi_rotor_landing_sim/src/training_q_learning/src/training_q_learning/parameters.py)  file with the following values
```
self.done_criteria = {
                "max_lon_distance" : True,  #Bool
                "max_lat_distance" : True,  #Bool
                "max_ver_distance" : True,  #Bool
                "max_num_timesteps" : False, #Bool
                "touchdown" : True, #Bool
                "success" : False,   #Bool
        }
self.init_distribution = 'uniform'   
self.init_height = 2.4 
```
Run the script 
```
cd path/to/directory
bash src/training_q_learning/scripts/training/launch_test_model_2D.sh hummingbird 11311 11351
```
You can run an analyis script in parallel. In that [script](rl_multi_rotor_landing_sim/src/training_q_learning/scripts/test_model_2D.py), adapt the file location and platform size. Then, execute
```
cd path/to/directory
src/training_q_learning/scripts/training/launch_analysis_node_2D.sh hummingbird 11311 11351
```
You can use one of the scripts provided in the folder [experiment_evaluation](experiment_evaluation)  to determine the [success rate](experiment_evaluation/success_determination.py) or [duration of training](experiment_evaluation/analyze_training_time.py) of the agent. 

### Run custom training
There are several files that need to be updated when you set up your own training scenario. On the one hand, there is the [parameters](rl_multi_rotor_landing_sim/src/training_q_learning/src/training_q_learning/parameters.py)  file. On the other hand, there are the files that specifiy the boundaries of the discrete states resulting from the multi-resolution discretization. Please find them in the [config](rl_multi_rotor_landing_sim/src/training_q_learning/config) directory. Only the files associated with the discretization in x and y direction need to be modified since only those are used in the current state of the code. They expect a list of normalized values in ascending order. To compute the boundary values you can either follow the equations given in the paper or run the [MATLAB scripts](rl_multi_rotor_landing_sim/other_files/matlab_scripts).

### Test an agent on real hardware
Coming soon...

