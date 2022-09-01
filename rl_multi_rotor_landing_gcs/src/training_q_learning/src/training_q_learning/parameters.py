import numpy as np
from typing import Any, Dict, List


'''
###############################################################################################
Never change the content of this file while a training is running. Some functions call
the parameters class during runtime, which would mean that the modified values would be loaded.
Training would not be consistent anymore.
###############################################################################################
'''

#Important parameters
M_PI = 3.14159265
f_agent = 22.91
episode_length = 20#s
training_days = 2
q_learning_algorithm = "double_q_learning"


class UAVParameters():
    def __init__(self):
        """Class provides data about the vehicle"""
        self.drone_name = "copter" #name of drone. Needs to match the one used to start up the environment.

        self.accel_cut_off_freq = 0.3 #hz, used in first order butterworth filter to compute the relative acceleration values based on the derivation of the relative velocity

        #Initial action values taken by the multi-rotor vehicle at the beginning of each episode
        self.initial_action_values = {"pitch":0,    #rad
                                      "roll":0,     #rad
                                      "v_z":-0.1,   #m/s
                                      "yaw":M_PI/4     #rad
                                      }      
        #Actions available to the agent
        self.action_strings =  {0:"increase_pitch",
                                1:"decrease_pitch",
                                2:"do_nothing"}     #do nothing needs to be last entry

        #Maximum pitch angle
        self.action_max_values = {"pitch":(M_PI/180)*5.589, #rad
                                  }
        #Maximum roll angle      
        self.action_delta_values = {"pitch":(M_PI/180)*1.863, #rad
                                  }                          

        #Observations considered by the agent 
        self.observation_msg_strings = {0:"rel_p_x", 
                                        1:"rel_v_x", 
                                        2:"rel_a_x", 
                                        } 

        #Observation values used for normalization
        self.observation_max_values = {"rel_p_x":1, #[m], float
                                       "rel_v_x":0.5657, #[m/s], float
                                       "rel_a_x":0.133, #[m/s^2], float
                                        }
        return
 

class RLParameters():
    def __init__(self):
        """Class provides variables that affect the reinforcement learning algorithm and structure of the learning task."""
        # Load training------------
        #Load training from
        self.load_data_from = "/home/frg_user/Desktop/vmp_0_4_rmp_0_5_fag_22_91/sim_3/training_results/training_q_learning_4/episode_97_FINAL"
        
        #Define location where the file is stored that contains the initial drone position at the beginning of each episode..
        self.store_logged_init_values_at = "automatic" # Options: 'automatic' - stores data in training log directory; 'some_absolute_path_to_folder' - stores values at the specified location; None / False - values are not stored        
        
        #Proceed training from last episode
        self.proceed_from_last_episode = False # False - new training is started | True - training is resumed from specified episode
        
        #Number of new refinement steps
        self.add_refinement_step = 1 # Int - how many ref. steps are added
        
        #List of tables whose values are copied and serve as initial values for the next round of training
        self.copy_table_list = ["Q_table","Q_table_double","state_action_counter"] 
        
        #List of tables whose values are scaled by scale modification value    
        self.scale_table_values_of = ["Q_table","Q_table_double"]  
        self.scale_modification_value = {} 
        self.scale_modification_value["Q_table"] = [0.8173, 0.8211, 0.8257, 0.8312]  # List of floats  - Values specifying the scale factors applied to the values of the copied tables                                  
        self.scale_modification_value["Q_table_double"] = [0.8173, 0.8211, 0.8257, 0.8312]  # List of floats  - Values specifying the scale factors applied to the values of the copied tables                   
        self.scaling_mode = 'scale' # String, mode of scaling: "scale" - all values are scaled by the factor spcecified in scale_modification_value and that applies to the specifiy ref. step | "scale_clip" - The values in the table are scaled in such a way that the highest value in the table takes the value specified by scale_modification_value and the remaining values are scaled in a way maintaining the ration of the unscaled values to each other        
        
        #List of tables whose values are clipped at a predefined integer value (only for tables containing integer values)
        self.reset_table_values = {  # Dict of table_name and int - clips the values of the table to the specified integer value
            }
        
        #The state action counter is clipped at this value    
        self.max_number_of_state_action_visits = 1000 #int
        


        #Training -------------------------
        #Latest curriculum step
        self.curriculum_step =  3  # Int, indexing starts with 0

        #Number of episodes taken into consideration to determine the percentage of successful episodes.
        self.number_of_successful_episodes = 100 #Int
        self.successful_fraction = 0.95      #Float - If set to 0, number_of_successful_episodes is the number of episodes that need to be successful consequtively.

        #Duration spend in the latest refinement step without interruption before success is triggered.
        self.ref_step_success_duration = 1*f_agent

        #Discretization mode
        self.discretization_mode = 'dynamic_model' 

        #Number of intervals used to discretize an observation of the environment within the limits of the value range
        self.n_r = 3 #int
    
        #Agent frequency
        self.running_step_time = 1/f_agent  #Float
        
        #Maximum number of timesteps in training
        self.max_num_timesteps = int(1/(self.running_step_time)*3600*24*training_days) #Int
        
        #Maximum number timesteps per episode
        self.max_num_timesteps_episode = int(f_agent*episode_length) #float

        #Maximum number of episodes
        self.max_num_episodes = 50000 #int
        
        #Schedule for the learning rate
        self.learning_rate = 'adaptive' # 'adaptive' - Learning rate is based on the number of visits of a state action pair; Dict with structure of exploration rate schedule. 
        self.omega = 0.51

        #Discount factor
        self.gamma = 0.99

        #Schedule for the exploration rate
        self.exploration_rate_schedule = {
            0:["lin",0,800,0,0]                                     
                                         }  #Dict of ints and floats
        #Initial exploration rate                                                                                   
        self.exploration_initial_eps = 0

        #Seed for numpy random number generators. Used for determinating the initial position of the drone and the moving platform at the beginning of each episode as well as for the epsilon greedy policy
        self.seed_init = 113 #Int
 

        #Data logging ---------------------------
        #Interval between episodes that are saved
        self.episode_save_freq = 1  #Int

        #Intervals between episodes after which training statistics are printed to the console
        self.print_info_freq = 1    #Int

        #Number of episodes that are used for averaging values
        self.print_info_mean_number = 1 #Int

        #Enable printing of statistics to the console
        self.verbose = 1    #1 - enabled; 0 - off
        
        #Algorithm used to perform the training
        self.q_learning_algorithm = q_learning_algorithm 
        return

class SimulationParameters(UAVParameters):
    def __init__(self):
        """Class provides parameter affecting the training and values generated from the environment"""
        super().__init__() 
        #Mode used to initialize the drone's position at the beginning of each episode.
        self.init_mode = 'absolute' # 'absolute' - drone is initialized in value range specified w.r.t. entire flying area; 'relative' - drone is initialized in value range w.r.t. current position of the moving platform

        #Probability distribution used to determine the initial position of the drone
        self.init_distribution = 'normal'   # 'normal' - normal distribution with mean and std, 'uniform' - uniform distribution

        #Parameters for normal distribution
        self.init_mu_x = 0 #float
        self.init_sigma_x = 0.33 #float
        self.init_mu_y = 0 #float
        self.init_sigma_y = 0#float

        #Parameters for uniform distribution. A section on the negative and the positive real axis can be specified
        self.init_max_x = np.array([0,1])     #array of floats
        self.init_min_x = np.array([-1,0])    #array of floats
        self.init_max_y = np.array([0,1])     #array of floats
        self.init_min_y = np.array([-1,0])    #array of floats
        self.init_max_z = np.array([0,0])       #array of floats
        self.init_min_z = -np.array([0,0])      #array of floats

        #Initial height
        self.init_height = 4 #[m] float

        #Parameters for the moving platform
        self.touch_down_height = 0.2 #[m] float

        #Reward definition
        self.grad_start_rel_p = -100
        self.grad_start_rel_v = -10
        self.action_weight = -1.55
        self.reward_factor_pos = 1
        self.reward_ref_step_decrease = -3
        self.reward_duration = -6.0*(1/f_agent)

        #Rewards for terminal conditions
        self.reward_leave_fly_zone = -3 #multiple of the maximum reward possible in one timestep while being in the current refinement step
        self.reward_success = 3 #multiple of the maximum reward possible in one timestep while being in the current refinement step
        self.max_timestep_in_episode = -3 #multiple of the maximum reward possible in one timestep while being in the current refinement step

        #Terminal condtions activation
        self.done_criteria = {
                "max_lon_distance" : True,  #Bool
                "max_lat_distance" : True,  #Bool
                "max_ver_distance" : True,  #Bool
                "max_num_timesteps" : False, #Bool
                "touchdown" : True, #Bool
                "success" : False,   #Bool
        }

        #Definition of terminal conditions
        #Mode applied for definitio  of terminal condtions
        self.position_terminal_mode = "absolute"    # 'absolute' - parameters below denote the distance to the earth fixed frame's origin; 'relative' - parameters below denote the distance to the moving platform
        self.max_abs_p_x = 1 #[m] float
        self.max_abs_p_y = 1 #[m] float
        self.max_abs_p_z = 10 #[m] float
        self.max_abs_rel_yaw = M_PI #[rad] float


class Parameters():
    def __init__(self):
       self.uav_parameters = UAVParameters()
       self.rl_parameters = RLParameters()
       self.simulation_parameters = SimulationParameters()
       return