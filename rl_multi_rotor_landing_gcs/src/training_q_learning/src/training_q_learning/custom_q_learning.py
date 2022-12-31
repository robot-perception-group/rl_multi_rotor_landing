'''
This script defines a class that contains all necessary methods to perform the training, to load data of previous training rounds and to predict actions of a trained agent.
'''

import gym
import numpy as np
import rospkg
import os
import shutil
import time
import pickle
from training_q_learning.utils import create_log_dir_path
from training_q_learning.parameters import Parameters
from training_q_learning.landing_simulation_env import LandingSimulationEnv
from training_q_learning.utils_multiresolution import get_discrete_state_from_ros_msg,initialize_grid_list,add_cur_step_lims_of_state
from copy import deepcopy
from training_q_learning.utils_q_learning import LogTraining
from training_q_learning.utils_q_learning import decay_rate_from_schedule


#Class definition
class QLearning():
    def __init__(self):
        """Class contains different functions to perform RL training for landing a multi-rotor vehicle on a moving platform."""
        #Read parameters and assign to class variables
        self.parameters = Parameters()
        self.learning_rate = self.parameters.rl_parameters.learning_rate
        self.gamma = self.parameters.rl_parameters.gamma
        self.max_num_timesteps_episode = self.parameters.rl_parameters.max_num_timesteps_episode
        self.max_num_episodes = self.parameters.rl_parameters.max_num_episodes
        self.n_r = self.parameters.rl_parameters.n_r
        self.running_step_time = self.parameters.rl_parameters.running_step_time
        self.exploration_initial_eps = self.parameters.rl_parameters.exploration_initial_eps
        self.omega = self.parameters.rl_parameters.omega
        self.save_freq = self.parameters.rl_parameters.episode_save_freq
        self.print_freq = self.parameters.rl_parameters.print_info_freq
        self.verbose = self.parameters.rl_parameters.verbose 
        self.mean_number = self.parameters.rl_parameters.print_info_mean_number
        self.number_of_successful_episodes = self.parameters.rl_parameters.number_of_successful_episodes
    
        #Define other variables
        np.random.seed(self.parameters.rl_parameters.seed_init)
        self.node_name = 'landing_simulation_gym_node'
        self.names_of_tables_to_save = []
        self.e = 0
        self.number_of_successful_episodes = 0
        self.successful_episodes_array = np.zeros(self.parameters.rl_parameters.number_of_successful_episodes)
        return

    def load_training(self):
        '''
        Function overwrites the init values with saved values of a saved training
        '''
        #if not empty:
        if self.parameters.rl_parameters.load_data_from:
            base_path = self.parameters.rl_parameters.load_data_from
            logged_parameters_path = base_path + "_logged_parameters.pickle"
            with open(logged_parameters_path,"rb") as callback_file:
                loaded_logged_parameters = pickle.load(callback_file)
                self.parameters_restarted_session  = loaded_logged_parameters["parameters"]
                print("Successfully loaded parameters from: ",logged_parameters_path)
                self.learning_rate = self.parameters_restarted_session.rl_parameters.learning_rate
                print("Succesfully loaded learning rate: ",self.learning_rate)
                self.gamma = self.parameters_restarted_session.rl_parameters.gamma
                print("Succesfully loaded gamma: ",self.gamma)
                self.max_num_timesteps_episode = self.parameters_restarted_session.rl_parameters.max_num_timesteps_episode
                print("Succesfully loaded max_num_timesteps_episode: ",self.max_num_timesteps_episode)
                self.n_r = self.parameters_restarted_session.rl_parameters.n_r
                print("Succesfully loaded n_r: ",self.n_r)
                self.running_step_time = self.parameters_restarted_session.rl_parameters.running_step_time
                print("Succesfully loaded running_step_time: ",self.running_step_time)
                self.names_of_tables_to_save = loaded_logged_parameters["names_of_tables_to_save"]
                print("Succesfully loaded names_of_tables_to_save: ",self.names_of_tables_to_save)
                if self.parameters.rl_parameters.proceed_from_last_episode == True:
                    self.e = loaded_logged_parameters["episode_number"]
                    print("The training will be proceeded from episode",self.e)
                    self.exploration_rate_schedule = loaded_logged_parameters["parameters"].rl_parameters.exploration_rate_schedule
                    print("Succesfully loaded last recorded exploration rate schedule: ",self.exploration_rate_schedule)
                    self.loaded_last_episode_reward = loaded_logged_parameters["episode_reward"]
                    print("Succesfully loaded last recorded episode reward: ",self.loaded_last_episode_reward)
                    self.loaded_last_episode_reward_array = loaded_logged_parameters["episode_reward_array"]
                    print("Succesfully loaded episode reward array... ")
                    self.loaded_last_episode_length_array = loaded_logged_parameters["episode_length_array"]
                    print("Succesfully loaded episode length array...")
                    self.loaded_episode_number = loaded_logged_parameters["episode_number"]
                    print("Succesfully loaded episode_number:",self.loaded_episode_number)
                    self.exploration_initial_eps = self.parameters_restarted_session.rl_parameters.exploration_initial_eps
                    print("Succesfully loaded exploration initial eps: ",self.exploration_initial_eps)
                    self.loaded_last_exploration_rate = loaded_logged_parameters["exploration_rate"]
                    print("Succesfully loaded exploration rate: ",self.loaded_last_exploration_rate)
                    self.loaded_done_numeric_1 = loaded_logged_parameters["done_numeric_1"]
                    print("Successfully loaded done_numeric_1",self.loaded_done_numeric_1)
                    self.loaded_done_numeric_2 = loaded_logged_parameters["done_numeric_2"]
                    print("Successfully loaded done_numeric_2",self.loaded_done_numeric_2)
                    self.loaded_done_numeric_3 = loaded_logged_parameters["done_numeric_3"]
                    print("Successfully loaded done_numeric_3",self.loaded_done_numeric_3)
                    self.loaded_done_numeric_4 = loaded_logged_parameters["done_numeric_4"]
                    print("Successfully loaded done_numeric_4",self.loaded_done_numeric_4)
                    self.loaded_done_numeric_5 = loaded_logged_parameters["done_numeric_5"]
                    print("Successfully loaded done_numeric_5",self.loaded_done_numeric_5)
                    self.loaded_done_numeric_6 = loaded_logged_parameters["done_numeric_6"]
                    print("Successfully loaded done_numeric_6",self.loaded_done_numeric_6)
                    self.loaded_done_numeric_7 = loaded_logged_parameters["done_numeric_7"]
                    print("Successfully loaded done_numeric_7",self.loaded_done_numeric_7)
                    self.loaded_done_numeric_8 = loaded_logged_parameters["done_numeric_8"]
                    print("Successfully loaded done_numeric_8",self.loaded_done_numeric_8)
                    self.loaded_Q_table_mean_value = loaded_logged_parameters["Q_table_mean_value"]
                    print("Successfully loaded Q table mean value",self.loaded_Q_table_mean_value)
                    self.loaded_number_not_visited_state_action_pairs = loaded_logged_parameters["number_not_visited_state_action_pairs"]
                    print("Successfully loaded number of not visited state-action pairs",self.loaded_number_not_visited_state_action_pairs)
                    self.loaded_training_successful_fraction = loaded_logged_parameters["training_successful_fraction"]
                    print("Successfully loaded training_successful_fraction",self.loaded_training_successful_fraction)
                    self.loaded_number_of_successful_episodes = loaded_logged_parameters["number_of_successful_episodes"]
                    self.number_of_successful_episodes = loaded_logged_parameters["number_of_successful_episodes"]
                    print("Successfully loaded number_of_successful_episodes",self.loaded_number_of_successful_episodes)
                    self.loaded_successful_episodes_array = deepcopy(loaded_logged_parameters["successful_episodes_array"])
                    self.successful_episodes_array = deepcopy(loaded_logged_parameters["successful_episodes_array"])
                    print("Successfully loaded successful_episodes_array",self.successful_episodes_array)
                    #Consider the possibility of a modified number of successful episodes when new curriculum step is added
                    if len(self.successful_episodes_array) < self.parameters.rl_parameters.number_of_successful_episodes:
                        padding_array = np.zeros(self.parameters.rl_parameters.number_of_successful_episodes - len(self.successful_episodes_array))
                        self.successful_episodes_array = np.concatenate((self.successful_episodes_array,padding_array))
                        print("Adapted length of array of successful episodes to match specified number of of successful episodes used for determining the successful fraction.")
                else:
                    print("The training will be not proceeded and start from episode",self.e)
                    print("Exploration_initial_eps: ",self.exploration_initial_eps)
                
                #Load the saved tables
                for table_name in self.names_of_tables_to_save:
                    #load table
                    table = deepcopy(np.load(base_path +"_"+ table_name + ".npy"))
                    setattr(self,table_name,table)
                    print("Successfully loaded table: ",table_name)
        return loaded_logged_parameters
                
    def scale_q_values(self,q_table:np.ndarray,modification_value,mode:str = "scale_clip"):
        '''
        Function scales the values of a Q-table. Two options are available.
        - Mode scale_clip:
            Function scales the Q-values of each state in such a way that the Q-value with the highest absolute value of any of the available actions takes the value modification_value while the ratio to the other Q-values is maintained.
        - Mode scale:
            Q table is scaled with the factor modification_value
        '''
        if mode == "scale_clip":
            for idx in np.ndindex(q_table.shape[:-1]):
                if any(q_table[idx]):                 
                    max_q_value = np.amax(np.abs(q_table[(idx)]))
                    unscaled_q_values = deepcopy(q_table[idx])
                    if max_q_value == 0:
                        scale_factor = 1
                    else:
                        scale_factor = modification_value/max_q_value
                    q_table[idx] = np.multiply(unscaled_q_values,scale_factor)
                    if np.any(np.isnan(q_table[idx])):
                        print("Error: Scaling led to nan")
                        print("scale_factor = ",scale_factor)
                        print("idx = ",idx)
                        print("q_table[idx] = ",q_table[idx])
                        raise ValueError
        elif mode == "scale":
            q_table = deepcopy(modification_value*q_table)
        return q_table

    def append_multiple_curriculum_steps_to_curriculum_sequence(self,num_ref_steps:int):
        """Function adds num_ref_steps new curriculum steps to the sequential curriculum and updates all related tables (e.g. Q-tables) accordingly."""
        self.lims_of_cur_steps = dict()

        #Determine all limit value for state discretization up to the current curriculum step
        for state in self.parameters.uav_parameters.observation_msg_strings.values(): 
            add_cur_step_lims_of_state(state,self.lims_of_cur_steps,parameters=self.parameters)
        
        #Iterate through all saved tables
        for table_name in self.names_of_tables_to_save:
            table = getattr(self,table_name)
            print("Table info: ",table_name)
            print("Shape before insert",table.shape)

            #Get index of most recent curriculum step before before the insertion of new curriculum steps
            idx_latest_cur_step_before_insert = len(table) - 1

            #Insert new blocks for the table
            for i in range(1,num_ref_steps+1):
                print("Insert new curriculum step no.",i)
                table = np.insert(table,len(table),0.0,axis = 0)
                print("Shape after inserting new curriculum step no.",i,": ",table.shape)
                
                #Check if the selected curriculum step is valid or not.
                assert len(table) <= len(self.parameters.rl_parameters.discretization_steps[self.parameters.uav_parameters.observation_msg_strings[0]]),"Specified curriculum step not valid. Max. valid curriculum step: "+str(len(self.parameters.rl_parameters.discretization_steps[self.parameters.uav_parameters.observation_msg_strings[0]]))

                #Store in class variables
                setattr(self,table_name,table)

                #Perform the transfer of knowledge to the newly added curriculum step's table values
                if table_name in self.parameters.rl_parameters.copy_table_list: 
                    #Pass  knowledge learned in previous refinement step to new refinement step.
                    idx_new_ref_step = len(table)-1
                    table_new_cur_step = deepcopy(table[idx_new_ref_step - 1])

                    #If the values need to be scaled for that table
                    if table_name in self.parameters.rl_parameters.scale_table_values_of:
                        scale_modification_value = self.parameters.rl_parameters.scale_modification_value[table_name][idx_latest_cur_step_before_insert+i-1]
                        mode = self.parameters.rl_parameters.scaling_mode 
                        table_new_cur_step = self.scale_q_values(table_new_cur_step,scale_modification_value,mode = mode)
                        print("Scaled values of subpart that was added to table:",table_name,", scaling mode:",mode,", scale_factor:",scale_modification_value)
                    table[idx_new_ref_step] = deepcopy(table_new_cur_step)
                    print("Copied values")
            print("-------------------")

        #Plausibility check
        for state in self.parameters.uav_parameters.observation_msg_strings.values(): 
            print(self.lims_of_cur_steps)
            assert len(table) == len(self.lims_of_cur_steps[state]),"The curriculum step specified for training is not correctly set (number of available Q-tables does not match the specified curriculum step)."
        return   

    def reset_table_values_to_value(self,table_name:str,value:int):
        """Function resets table values to predefined integer value."""
        table = getattr(self,table_name)
        table_clipped = np.clip(table,None,int(value))
        setattr(self,table_name,table_clipped)
        return

    def initialize_table(self,table_name:str,init_value:float,save_table:bool = True):
        '''
        Function creates a numpy.ndarray that can be indexed according to a multidimensional grid scheme.
        The grid vectors are defined by the number of discrete relative position, velocity, acceleration states and discrte action states as well as the  
        number of actions considered for the landing problem.
        '''
        #Create initial empty grid vectors
        grid = [] 

        #Add grid vector for indicating the used refinement step / curriculum step and give it the initial value of 0
        grid.append([0])

        #Add relative state grid vectors
        for obs_msg_string in self.parameters.uav_parameters.observation_msg_strings:
            grid.append(list(np.arange(self.n_r)))

        #Add action value grid vectors
        for act_string in self.parameters.uav_parameters.action_max_values.keys():
            max_value = self.parameters.uav_parameters.action_max_values[act_string]
            delta_value = self.parameters.uav_parameters.action_delta_values[act_string]
            grid.append(list(np.arange(np.rint(2*max_value/delta_value + 1)))) # +1 because we need to consider the value at -max_value
        
        #Add action choice grid vector
        grid.append(list(np.arange(len(self.parameters.uav_parameters.action_strings))))

        #Initialize table
        table = initialize_grid_list(grid,init_value)

        #Save in QLearning variables
        setattr(self,table_name,table)

        #Mark table as to be saved if requested
        if save_table == True: self.names_of_tables_to_save.append(table_name)

        print("Initialized table",table_name,"with init value",str(init_value),"and save property set to",str(save_table))
        return

    def setup_logging_system(self):
        '''
        Function sets up the logging system for the training environment. It handles the saving of the training, generates the output to the terminal and display of training parameters with tensorboard.
        '''
        #Create folder where to store the logged data
        rospack = rospkg.RosPack()
        package_name = "training_q_learning"
        log_dir_path = create_log_dir_path(os.path.join(rospack.get_path(package_name), 'training_results'),package_name) #Create the path to a folder in which the current model as well as its checkpoints will be stored
        print("setup_logging_system: path to package training_q_learning = ",log_dir_path)
        
        #prepare folder for storing data
        if os.path.exists(log_dir_path):
            shutil.rmtree(log_dir_path)
        os.makedirs(log_dir_path)
        self.save_path_base = os.path.join(log_dir_path, 'model_episode_')

        #If the training is not a loaded one or if a loaded training should not proceed from the last saved episode then some values need to be 
        #initialized which are otherwise initialized by loaded values.
        #Parameters whose starting values are independent of saved values
        self.training_log_data = dict()
        self.training_log_data["episode_reward"] = 0
        self.training_log_data["loss"] = 0
        self.training_log_data["max_episode_loss"] = 0
        self.training_log_data["parameters"] = self.parameters
        if not self.parameters.rl_parameters.load_data_from or not self.parameters.rl_parameters.proceed_from_last_episode:
            print("Set initial values for data logging")
            self.training_log_data["exploration_rate"] = self.exploration_initial_eps
            self.training_log_data["episode_reward_array"] = []
            self.training_log_data["episode_length_array"] = []
            self.training_log_data["episode_number"] = 0
            self.training_log_data["done_numeric_1"] = 0
            self.training_log_data["done_numeric_2"] = 0
            self.training_log_data["done_numeric_3"] = 0
            self.training_log_data["done_numeric_4"] = 0
            self.training_log_data["done_numeric_5"] = 0
            self.training_log_data["done_numeric_6"] = 0
            self.training_log_data["done_numeric_7"] = 0
            self.training_log_data["done_numeric_8"] = 0
            self.training_log_data["Q_table_mean_value"] = 0
            self.training_log_data["number_not_visited_state_action_pairs"] = 0
            self.training_log_data["reward"] = 0
            self.training_log_data["training_successful_fraction"] = 0
            self.training_log_data["number_of_successful_episodes"] = self.number_of_successful_episodes
            self.training_log_data["successful_episodes_array"] = self.successful_episodes_array 
        else:
            #Parameters whose starting values depend on saved values
            self.training_log_data["exploration_rate"] = self.loaded_last_exploration_rate
            self.training_log_data["episode_reward_array"] = deepcopy(self.loaded_last_episode_reward_array)
            self.training_log_data["episode_length_array"] = deepcopy(self.loaded_last_episode_length_array)
            self.training_log_data["episode_number"] = self.loaded_episode_number
            self.training_log_data["done_numeric_1"] = self.loaded_done_numeric_1
            self.training_log_data["done_numeric_2"] = self.loaded_done_numeric_2
            self.training_log_data["done_numeric_3"] = self.loaded_done_numeric_3
            self.training_log_data["done_numeric_4"] = self.loaded_done_numeric_4
            self.training_log_data["done_numeric_5"] = self.loaded_done_numeric_5
            self.training_log_data["done_numeric_6"] = self.loaded_done_numeric_6
            self.training_log_data["done_numeric_7"] = self.loaded_done_numeric_7
            self.training_log_data["done_numeric_8"] = self.loaded_done_numeric_8
            self.training_log_data["Q_table_mean_value"] = self.loaded_Q_table_mean_value
            self.training_log_data["number_not_visited_state_action_pairs"] = self.loaded_number_not_visited_state_action_pairs
            self.training_log_data["reward"] = 0
            self.training_log_data["training_successful_fraction"] = self.loaded_training_successful_fraction
            self.training_log_data["number_of_successful_episodes"] = self.loaded_number_of_successful_episodes
            self.training_log_data["successful_episodes_array"] = self.loaded_successful_episodes_array

        #Load saved tables
        for table_name in self.names_of_tables_to_save:
            self.training_log_data[table_name] = getattr(self,table_name)

        #Initialize logging callback
        self.log_training = LogTraining(self.training_log_data,self.names_of_tables_to_save,self.print_freq,self.save_freq,self.mean_number,log_dir_path,self.verbose)
        
        #Store the path where data is logged for usage elsewhere
        self.env.log_dir_path = log_dir_path
        return

    def init_training_env(self, reset_length:int ):
        '''
        Function sets up the training environment and performs intial reset
        '''
        print("Starting init procedure of training environment...")
        self.env = gym.make('landing_simulation-v0')
        self.env.reset()
        self.env.unpause_sim()
        time.sleep(reset_length)
        self.env.pause_sim()
        self.env.reset()
        self.env.unpause_sim()
        time.sleep(reset_length)
        self.env.pause_sim()
        print("Init procedure of training environment completed.")
        return

    def double_q_learning(self):
        '''
        Function implements the Double Q-learning algorithm presented in https://proceedings.neurips.cc/paper/2010/file/091d584fced301b442654dd8c23b3fc9-Paper.pdf
        '''
        #Initialize the dict that contains the limit values for the different states and curriculum steps
        self.lims_of_cur_steps = dict()
        for state in self.parameters.uav_parameters.observation_msg_strings.values(): 
            add_cur_step_lims_of_state(state,self.lims_of_cur_steps)        
            assert len(self.lims_of_cur_steps[state]) == len(self.Q_table),"The index for the most recently added curriculum step that is specified in the parameters file does not match the number of curriculum steps stored in the Q-tables (expecting" + str(self.Q_table.shape[0]-1) + ")."
        
        #Make variables available to landing simulation object class
        self.env.landing_simulation_object.lims_of_cur_steps = deepcopy(self.lims_of_cur_steps)
        self.env.landing_simulation_object.n_r = self.parameters.rl_parameters.n_r
        self.training_log_data["lims_of_cur_steps"] = self.lims_of_cur_steps
        print("Done setting up the limit values for multi-resolution discretization.")

        #Determine initial value for exploration rate
        if not self.parameters.rl_parameters.load_data_from or not self.parameters.rl_parameters.proceed_from_last_episode:
            self.exploration_rate = self.exploration_initial_eps
        else:
            self.exploration_rate = self.loaded_last_exploration_rate
        self.training_log_data["exploration_rate"] = self.exploration_rate

        #Enter the training loop
        first_training_step = True
        while self.e <= self.max_num_episodes:
            #Get initial observation
            S_0_env = self.env.reset()
            
            #Discretize observation
            S_0 = get_discrete_state_from_ros_msg(S_0_env[0],S_0_env[1],self.lims_of_cur_steps,self.n_r,self.parameters)
            S_t = S_0

            #Set episode done marker to false
            done = False
            
            #Iterate through the episodes
            for i in range(self.max_num_timesteps_episode):

                #Choose random action or exploit Q-values
                if np.random.uniform(0,1) < self.exploration_rate:            
                    A_t = np.random.randint(len(self.parameters.uav_parameters.action_strings))
                else:
                    #Form the mean of the two Q-value arrays and determine the action idx with the highest Q-value associated
                    A_t = np.argmax(np.add(self.Q_table[S_t],self.Q_table_double[S_t])/2)

                self.env.landing_simulation_object.previous_state_idx = S_t

                #Perform one training step
                S_t_1_env, R, done, _ = self.env.step(A_t)
                #Get Q table indices of next state
                S_t_1 = get_discrete_state_from_ros_msg(S_t_1_env[0],S_t_1_env[1],self.lims_of_cur_steps,self.n_r,self.parameters)   
                
                #Determine the idx of the current state action pair
                S_t_A_t = S_t +(A_t,)

                #Determine the learning rate
                if self.parameters.rl_parameters.learning_rate == 'adaptive':
                    alpha = np.max([(1/(self.state_action_counter[S_t_A_t]+1))**self.omega,self.parameters.rl_parameters.alpha_min])
                elif type(self.parameters.rl_parameters.learning_rate) is dict:
                    alpha = decay_rate_from_schedule(self.e,self.parameters.rl_parameters.learning_rate,None)
                else:
                    alpha = self.parameters.rl_parameters.learning_rate

                #Decide which Q_table to update
                if np.random.randint(0,2) == 0: #Update Q_table A, according to algorithm
                    A_star = np.argmax(self.Q_table[S_t_1])
                    S_t_1_A_star = S_t_1 + (A_star,)
                    if S_t_1[0] == S_t[0]:    
                        #Non-terminal state
                        loss = alpha*(R+self.gamma*self.Q_table_double[S_t_1_A_star]-self.Q_table[S_t_A_t])
                    else:
                        #Terminal state: The expected sum of future rewards equals zero
                        loss = alpha*(R + 0 - self.Q_table[S_t_A_t])
                    self.Q_table[S_t_A_t] = self.Q_table[S_t_A_t] + loss

                else: #Update Q_table B, according to algorithm
                    B_star = np.argmax(self.Q_table_double[S_t_1])
                    S_t_1_B_star = S_t_1 + (B_star,)
                    if S_t_1[0] == S_t[0]:   
                        #Non-terminal state 
                        loss = alpha*(R+self.gamma*self.Q_table[S_t_1_B_star]-self.Q_table_double[S_t_A_t])
                    else:
                        #Terminal state: The expected sum of future rewards equals zero
                        loss = alpha*(R + 0 - self.Q_table_double[S_t_A_t])
                    self.Q_table_double[S_t_A_t] = self.Q_table_double[S_t_A_t] + loss

                #Logging    
                self.training_log_data["loss"] = loss
                if abs(self.training_log_data["loss"]) > self.training_log_data["max_episode_loss"]: 
                    self.training_log_data["max_episode_loss"] = abs(self.training_log_data["loss"])

                #Update counter of visits of state action pair
                self.state_action_counter[S_t_A_t] += 1  
                                
                #Save data for logging
                if first_training_step == True:
                    first_training_step = False
                else:
                    self.training_log_data["reward"] = R
                self.training_log_data["touchdown_on_platform"] = self.env.touchdown_on_platform
                self.training_log_data["done_numeric"] = self.env.landing_simulation_object.done_numeric
                
                #Handle data logging for timestep
                self.log_training.on_step()

                # Leave for loop if episode is finished
                if done:
                    break
                S_t = deepcopy(S_t_1)

            #Check end criteria of training
            #Store successful or unsuccessful episodes in an array
            if not self.env.landing_simulation_object.done_numeric == 8:
                #handle counter
                self.number_of_successful_episodes = 0

                #shift values one position to the right
                self.successful_episodes_array[1:] = self.successful_episodes_array[:-1]

                #insert new value at beginning
                self.successful_episodes_array[0] = 0
            else:
                #handle counter
                self.number_of_successful_episodes += 1

                #shift values one position to the right
                self.successful_episodes_array[1:] = self.successful_episodes_array[:-1]

                #insert new value at beginning
                self.successful_episodes_array[0] = 1

            #Perform saving of episode specific data
            self.training_log_data["training_successful_fraction"] = np.sum(self.successful_episodes_array) / len(self.successful_episodes_array)
            self.training_log_data["number_of_successful_episodes"] = self.number_of_successful_episodes
            self.training_log_data["successful_episodes_array"] = deepcopy(self.successful_episodes_array)
            self.training_log_data["exploration_rate"] = self.exploration_rate

            #Perform logging qnd result saving as well as checking of terminal criteria of training
            self.log_training.on_episode_end()

            #Compute values for next episode
            self.e += 1
            self.exploration_rate = decay_rate_from_schedule(self.e,self.parameters.rl_parameters.exploration_rate_schedule,None)
            self.training_log_data["max_episode_loss"] = 0
        return


    def predict_action(self,S_t:tuple,verbose:bool = True):
        """Function samples actions based on the current Q-values."""
        q_values = np.add(self.Q_table[S_t],self.Q_table_double[S_t])/2
        A_t = np.argmax(q_values)
        if verbose == True:
            print(self.parameters.uav_parameters.action_strings[A_t])
        return A_t