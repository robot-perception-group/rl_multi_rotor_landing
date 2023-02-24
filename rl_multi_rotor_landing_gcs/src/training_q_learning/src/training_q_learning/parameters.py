'''
This script contains the parameters to set up a training case
###############################################################################################
Never change the content of this file while a training or evaluation is running. 
###############################################################################################
'''
import numpy as np

class UAVParameters():
    def __init__(self):
        """Class provides data about the vehicle"""
        self.accel_cut_off_freq: float = 0.3 #hz, used in first order butterworth filter to compute the relative acceleration values based on the derivation of the relative velocity

        #Initial action values taken by the multi-rotor vehicle at the beginning of each episode
        self.initial_action_values: dict = {"pitch":0,  #[rad]
                                      "roll":0,         #[rad]
                                      "v_z":-0.1,       #[m/s]
                                      "yaw":np.pi/4           #[rad]
                                      }

        #Actions available to the agent
        self.action_strings: dict =    { 0:"increase_pitch",
                                         1:"decrease_pitch",
                                         2:"do_nothing" #do nothing needs to be last entry
                                        }     

        #Maximum action values
        self.action_max_values: dict = {"pitch":np.deg2rad(5.58913), #rad
                                  }
        #Action increments      
        self.action_delta_values: dict = {"pitch":np.deg2rad(1.86304), #rad
                                  }                          

        #Observations considered by the agent 
        self.observation_msg_strings: dict = {0:"rel_p_x", #[m]
                                              1:"rel_v_x", #[m/s]
                                              2:"rel_a_x", #[m/s^2]
                                              } 

        #Observation values used for normalization
        self.observation_max_values: dict = {"rel_p_x":1,       #[m]
                                             "rel_v_x":0.8,   #[m/s]
                                             "rel_a_x":0.32,      #[m/s^2]
                                        }
        return
 

class RLParameters():
    def __init__(self):
        """Class provides variables that affect the reinforcement learning algorithm and structure of the learning task."""
        # Load training ---------------------------
        #Load training from
        self.load_data_from = "/home/pgoldschmid/Desktop/exp_paper_20230109/vmp_0_4_real/sim_3/training_results/training_q_learning_4/episode_96_FINAL"
        
        #Define location where the file is stored that contains the initial drone position at the beginning of each episode..
        self.store_logged_init_values_at: str = "automatic" # Options: 'automatic' - stores data in training log directory; 'some_absolute_path_to_folder' - stores values at the specified location; None / False - values are not stored        
        
        #Proceed training from last episode
        self.proceed_from_last_episode: bool = False # False - new training is started | True - training is resumed from last episode in loaded data
        
        #Number of new curriculum steps to be added to the sequential curriculum step
        self.number_new_curriculum_steps: int = 1 # how many curriculum steps are added
        
        #List of tables whose values are copied and serve as initial values for the next round of training
        self.copy_table_list: list = ["Q_table","Q_table_double","state_action_counter"] 
        
        #List of tables whose values are scaled to serve as a starting point for a new curriculum  
        self.scale_table_values_of: list = ["Q_table","Q_table_double"]  
        self.scale_modification_value:dict  = {"Q_table"        : [0.8172650252856599, 0.8211253690681617, 0.8257273369742982],
                                               "Q_table_double" : [0.8172650252856599, 0.8211253690681617, 0.8257273369742982]} 
        self.scaling_mode: str = 'scale' # String, mode of scaling: "scale" - values are scaled by the factor spcecified in scale_modification_value and that applies to the specified curriulum step | "scale_clip" - The new values aded to the table are scaled in such a way that the highest takes the value specified by scale_modification_value and the remaining values are scaled in a way maintaining the ratio of the unscaled values to each other        
        
        #List of tables whose values are clipped at a predefined integer value (only for tables containing integer values)
        self.reset_table_values: dict = {  # Dict of table_name and int - clips the values of the table to the specified integer value
            }
                

        #Training ---------------------------
        #Latest curriculum step
        self.curriculum_step: int = 3

        #Number of episodes taken into consideration to determine the percentage of successful episodes.
        self.number_of_successful_episodes: int = 100 

        #Fraction of the last number_of_successful_episodes that needs to terminate with success in order to finish the training
        self.successful_fraction: float = 0.96      #If set to 0, number_of_successful_episodes is the number of episodes that need to be successful consequtively.

        #Duration spend in the latest curriculum step without interruption before success is triggered.
        self.cur_step_success_duration: float = 1 #[s]

        #Discretization 
        self.discretization_steps: dict = {
            "rel_p_x" : [1.0, 0.64, 0.4096, 0.262144],
            "rel_v_x" : [1.0, 0.8, 0.64, 0.512],
            "rel_a_x" : [1.0, 1.0, 1.0, 1.0],
            "rel_p_y" : [1.0, 0.64, 0.4096, 0.262144],
            "rel_v_y" : [1.0, 0.8, 0.64, 0.512],
            "rel_a_y" : [1.0, 1.0, 1.0, 1.0]
        }
        self.beta_value: float = 1/3
        self.sigma_a: float = 0.416

        #Number of intervals used to discretize an observation of the environment within the limits of the value range
        self.n_r: int = 3 
    
        #Agent frequency
        self.f_ag: float = 22.92
        self.running_step_time: float = 1/self.f_ag  #[hz]
        
        #Maximum number of timesteps in training
        self.max_num_timesteps: int = int(self.f_ag*3600*24*2)
        
        #Maximum number timesteps per episode
        self.t_max: float = 20 #[s] 
        self.max_num_timesteps_episode: int = int(self.f_ag*self.t_max) 

        #Maximum number of episodes
        self.max_num_episodes: int = 50000 
        
        #Settings for the learning rate
        self.learning_rate = 'adaptive' # 'adaptive' - Learning rate is based on the number of visits of a state action pair; Dict with structure of exploration rate schedule. 
        self.omega: float = 0.51
        self.alpha_min: float = 0.02949

        #Discount factor
        self.gamma: float = 0.99

        #Schedule for the exploration rate
        self.exploration_rate_schedule: dict = {0:['lin',0,1,0,0]}
        #Initial exploration rate                                                                                   
        self.exploration_initial_eps: float = 0

        #Seed for the numpy random number generators. If None, numpy will determine the sequence based on /dev/urandom or the Windows analogue. Used for determinating the initial position of the drone and the moving platform at the beginning of each episode as well as for the epsilon greedy policy
        self.seed_init: int = None
 

        #Data logging ---------------------------
        #Interval between episodes that are saved
        self.episode_save_freq: int = 1  

        #Interval between episodes after which training statistics are printed to the console
        self.print_info_freq: int = 1    

        #Number of episodes that are used for averaging values
        self.print_info_mean_number: int = 20 

        #Enable printing of statistics to the console
        self.verbose: bool = True    #True - enabled; False - off
        
        #Algorithm used to perform the training
        self.q_learning_algorithm: str = "double_q_learning" 
        return

class SimulationParameters(UAVParameters):
    def __init__(self):
        """Class provides parameter affecting the training and values generated from the environment"""
        super().__init__() 
        #Mode used to initialize the drone's position at the beginning of each episode.
        self.init_mode: str = 'absolute' # 'absolute' - drone is initialized in the value range specified w.r.t. the entire flyzone; 'relative' - drone is initialized in value range w.r.t. the current position of the moving platform

        #Probability distribution used to determine the initial position of the drone
        self.init_distribution: str = 'uniform'   # 'normal' - normal distribution with mean and std, 'uniform' - uniform distribution

        #Parameters for normal distribution
        self.init_mu_x: float = 0
        self.init_sigma_x: float = 1/3 
        self.init_mu_y: float = 0 
        self.init_sigma_y: float = 0 

        #Parameters for uniform distribution. A section on the negative and the positive real axis can be specified
        self.init_max_x: np.array = np.array([0,1]) 
        self.init_min_x: np.array = np.array([-1,0])
        self.init_max_y: np.array = np.array([0,0]) 
        self.init_min_y: np.array = np.array([0,0])
        self.init_max_z: np.array = np.array([0,0])   
        self.init_min_z: np.array = np.array([0,0])  

        #Initial altitude
        self.init_altitude: float = 4 #[m]

        #Parameters for the moving platform
        self.minimum_altitude: float = 0.3 #[m]

        #Reward definition
        self.w_p: float = -100.0
        self.w_v: float = -10.0
        self.w_theta: float = -1.55
        self.w_dur: float = -6.0
        self.w_fail: float = -2.6
        self.w_suc: float = 2.6


        #Activation of terminal condtions
        self.done_criteria: dict = {
                "max_lon_distance" : False,  #Bool
                "max_lat_distance" : False,  #Bool
                "max_ver_distance" : False,  #Bool
                "max_num_timesteps" : False, #Bool
                "minimum_altitude" : False, #Bool
                "touchdown_contact" : False, #Bool
                "success" : False,   #Bool
        }

        #Definition of terminal conditions
        #Mode applied for definition of terminal condtions
        self.position_terminal_mode: str = "absolute"    # 'absolute' - parameters below denote the distance to the earth fixed frame's origin; 'relative' - parameters below denote the distance to the moving platform
        self.max_abs_p_x: float = 1 #[m] 
        self.max_abs_p_y: float = 1 #[m] 
        self.max_abs_p_z: float = 10 #[m] 
        self.max_abs_rel_yaw: float = np.pi #[rad] 


class Parameters():
    def __init__(self):
       self.uav_parameters = UAVParameters()
       self.rl_parameters = RLParameters()
       self.simulation_parameters = SimulationParameters()
       return