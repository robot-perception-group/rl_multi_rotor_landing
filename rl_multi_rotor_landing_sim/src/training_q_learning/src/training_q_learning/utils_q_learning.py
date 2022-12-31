"""
This script contains the definitions of classes and function that are required to perform the logging of data during training, the saving of results as well as the detemination of episode specific data.
"""

import numpy as np
import time,datetime,os
from torch.utils.tensorboard import SummaryWriter
import pickle


class LogTraining():
    def __init__(self,training_data:dict,names_of_tables_to_save:list,print_info_freq: int, episode_save_freq:int, mean_number: int,log_dir: str, verbose:bool=True):
        """Class contains functions for printing out training information and for data logging in tensorboard as well as for saving the results."""
        
        #Create tensorboard instance
        self.tb_writer = SummaryWriter(log_dir = log_dir)
       
        #Parameters for saving the model
        self.print_freq = print_info_freq
        self.save_freq = episode_save_freq
        self.episode_save_freq = episode_save_freq
        self.log_dir = log_dir
        self.save_path_base = os.path.join(log_dir, 'episode_')
        self.verbose = verbose

        #Parameters for data logging
        self.training_data = training_data
        self.parameters = self.training_data["parameters"]
        self.episode_reward_array = training_data["episode_reward_array"]
        self.episode_length_array = training_data["episode_length_array"]
        self.episode_number = training_data["episode_number"]
        self.t_start = time.time()
        
        #initialize data
        self.mean_number = mean_number
        self.names_of_tables_to_save = names_of_tables_to_save

        #Define the parameters that need to be logged, their values will be saved along with the data stored in tables
        self.logged_parameters = {
        "episode_number":self.episode_number,
        "episode_reward": training_data["episode_reward"],
        "step_number_in_episode" : 0,
        "touchdown_on_platform" : False,
        "done_numeric" : 0,
        "done_numeric_1":self.training_data["done_numeric_1"],
        "done_numeric_2":self.training_data["done_numeric_2"],
        "done_numeric_3":self.training_data["done_numeric_3"],
        "done_numeric_4":self.training_data["done_numeric_4"],
        "done_numeric_5":self.training_data["done_numeric_5"],
        "done_numeric_6":self.training_data["done_numeric_6"],
        "done_numeric_7":self.training_data["done_numeric_7"],
        "done_numeric_8":self.training_data["done_numeric_8"],
        "exploration_rate":self.training_data["exploration_rate"], #value should be updated whenevwer the value in training data for exploration rate changes
        "Q_table_mean_value":self.training_data["Q_table_mean_value"],
        "number_not_visited_state_action_pairs":self.training_data["number_not_visited_state_action_pairs"],
        "mean_number":self.mean_number,
        "episode_reward_array":self.episode_reward_array, #dict should be automatically updated when array changes
        "episode_length_array":self.episode_length_array, #dict entry should be automaticallly updated when array changes
        "parameters":self.training_data["parameters"],
        "names_of_tables_to_save":self.names_of_tables_to_save,
        "exploration_initial_eps":self.training_data["parameters"].rl_parameters.exploration_initial_eps,
        "exploration_rate_schedule":self.training_data["parameters"].rl_parameters.exploration_rate_schedule,
        "loss":self.training_data["loss"],
        "max_episode_loss":self.training_data["max_episode_loss"],
        "total_step_number":0,
        "training_successful_fraction":self.training_data["training_successful_fraction"],
        "number_of_successful_episodes":self.training_data["number_of_successful_episodes"],
        "successful_episodes_array":self.training_data ["successful_episodes_array"]
        }
        return
    
    def print_stats(self):
        '''Function prints info to the terminal when verbose is set to true.'''
        if self.verbose:
            if self.episode_number % self.print_freq == 0:
                if self.episode_number > self.mean_number:
                    episode_reward_mean = np.mean(self.episode_reward_array[-self.mean_number:])
                    episode_length_mean = np.mean(self.episode_length_array[-self.mean_number:])
                else:
                    episode_reward_mean = np.mean(self.episode_reward_array)
                    episode_length_mean = np.mean(self.episode_length_array)
                t_elapsed = time.time()-self.t_start
                print("Exploration probability: ",self.logged_parameters["exploration_rate"])
                print("Episode reward: ",self.episode_reward_array[-1])
                print("Episode length: ",self.episode_length_array[-1])
                print("Mean episode reward (last "+str(self.mean_number)+" episodes): ",episode_reward_mean)
                print("Mean episode length (last "+str(self.mean_number)+" episodes): ",episode_length_mean)
                print("Elapsed time: ",str(datetime.timedelta(seconds=int(t_elapsed))))
                print("Number of episodes: ",self.logged_parameters["episode_number"])
                print("Episode success counter: ",self.logged_parameters["done_numeric_8"])
                print("Terminal condition:",self.training_data["training_successful_fraction"]*100,"% (last",str(len(self.training_data["successful_episodes_array"])),"episodes)")
                print("=========================")
        return

    def save_training_data_tables(self,save_path_suffix = "",verbose = False):
        '''Function saves numpy.ndarrays that are used in the context of Q-learning. Array examples are the Q-table or the table that counts the visits for the state action pairs.'''
        for item in self.training_data.keys():
            if item in self.names_of_tables_to_save:        
                #save table
                np.save(self.save_path_base+str(self.episode_number)+"_"+save_path_suffix+"_"+item,self.training_data[item])
                if verbose:
                    print("Saved table "+item+ " here: "+self.save_path_base+str(self.episode_number)+"_"+item+".npy")
        return
    
    def save_training_data(self,save_path_suffix = "",save_now = False, verbose = False):
        '''Function saves the training and logging data'''
        if (self.episode_number % self.episode_save_freq == 0 and self.episode_number >= 1) or save_now == True: 
            self.save_logging_data(save_path_suffix = save_path_suffix,verbose = verbose)
            self.save_training_data_tables(save_path_suffix = save_path_suffix,verbose = verbose)
        return

    def save_logging_data(self,save_path_suffix = "",verbose = False):
        '''Function saves the logging data'''
        with open(self.save_path_base +str(self.episode_number)+"_"+save_path_suffix+ "_logged_parameters.pickle","wb") as save_file:
            pickle.dump(self.logged_parameters,save_file)
            if verbose:
                print("Saved logged parameters here: "+self.save_path_base+str(self.episode_number)+"_logged_parameters.pickle")
        return

    def on_episode_end(self):
        '''Function handles the logging of data at the end of each episode.'''
        #Compute values
        self.episode_number += 1
        self.logged_parameters["episode_number"] = self.episode_number
        self.logged_parameters["exploration_rate"] = self.training_data["exploration_rate"]
        self.episode_reward_array.append(self.logged_parameters["episode_reward"])
        self.episode_length_array.append(self.logged_parameters["step_number_in_episode"])
        if self.episode_number > self.mean_number:
            self.logged_parameters["episode_reward_mean"] = np.mean(self.episode_reward_array[-self.mean_number:])
            self.logged_parameters["episode_length_mean"] = np.mean(self.episode_length_array[-self.mean_number:])
        else:
            self.logged_parameters["episode_reward_mean"] = np.mean(self.episode_reward_array)
            self.logged_parameters["episode_length_mean"] = np.mean(self.episode_length_array)
        self.logged_parameters["Q_table_mean_value"] = np.mean(self.training_data["Q_table"])

        #Determine the number of not visited state action pairs
        self.logged_parameters["number_not_visited_state_action_pairs"] = np.sum(self.training_data["state_action_counter"] == 0)

        #Compute the mean, max and min of state action pair counter
        self.logged_parameters["state_action_counter_mean"] = np.mean(self.training_data["state_action_counter"])
        self.logged_parameters["state_action_counter_max"] = np.amax(self.training_data["state_action_counter"])
        self.logged_parameters["state_action_counter_min"] = np.amin(self.training_data["state_action_counter"])

        #Log data
        self.tb_writer.add_scalar("episode_info/episode_reward", self.logged_parameters["episode_reward"],global_step=self.logged_parameters["episode_number"])
        self.tb_writer.add_scalar("episode_info/episode_reward_mean",self.logged_parameters["episode_reward_mean"],global_step=self.logged_parameters["episode_number"])
        self.tb_writer.add_scalar("episode_info/number_of_episodes",self.logged_parameters["episode_number"],global_step=self.logged_parameters["episode_number"])
        self.tb_writer.add_scalar("episode_info/episode_length",self.logged_parameters["step_number_in_episode"],global_step=self.logged_parameters["episode_number"])
        self.tb_writer.add_scalar("episode_info/episode_length_mean",self.logged_parameters["episode_length_mean"],global_step=self.logged_parameters["episode_number"])
        self.tb_writer.add_scalar("episode_info/exploration_rate",self.logged_parameters["exploration_rate"],global_step=self.logged_parameters["episode_number"])
        self.tb_writer.add_scalar("done_criteria/max_num_timesteps_in_episode",self.logged_parameters["done_numeric_1"],global_step=self.logged_parameters["episode_number"])
        self.tb_writer.add_scalar("done_criteria/max_vert_dist",self.logged_parameters["done_numeric_2"],global_step=self.logged_parameters["episode_number"])
        self.tb_writer.add_scalar("done_criteria/max_long_dist",self.logged_parameters["done_numeric_3"],global_step=self.logged_parameters["episode_number"])
        self.tb_writer.add_scalar("done_criteria/max_lat_dist",self.logged_parameters["done_numeric_4"],global_step=self.logged_parameters["episode_number"])
        self.tb_writer.add_scalar("done_criteria/successful_episodes",self.logged_parameters["done_numeric_8"],global_step=self.logged_parameters["episode_number"])
        self.tb_writer.add_scalar("done_criteria/success_percentage",self.training_data["training_successful_fraction"]*100,global_step=self.logged_parameters["episode_number"])
        self.tb_writer.add_scalar("Q_table_info/mean_value_of_Q_table_A",self.logged_parameters["Q_table_mean_value"],global_step=self.logged_parameters["episode_number"])
        self.tb_writer.add_scalar("Q_table_info/number_not_visited_state_action_pairs",self.logged_parameters["number_not_visited_state_action_pairs"],global_step=self.logged_parameters["episode_number"])
        self.tb_writer.add_scalar("done_criteria/done_numeric",self.training_data["done_numeric"],global_step=self.logged_parameters["episode_number"])
        
        #save data if necessary
        self.save_training_data(save_path_suffix = "running")

        #print data if necessary
        self.print_stats() 

        #Check terminal condition of training
        #Possibility 1: End training based on performed episodes
        if self.parameters.rl_parameters.successful_fraction == 0 and self.training_data["number_of_successful_episodes"] == self.parameters.rl_parameters.number_of_successful_episodes:
            self.save_training_data(save_path_suffix = "FINAL",save_now = True,verbose = True)
            exit()
        #Possibility 2: End training based on successful fraction of last n episodes
        elif self.parameters.rl_parameters.successful_fraction > 0 and len(self.training_data["successful_episodes_array"]) >= self.parameters.rl_parameters.number_of_successful_episodes and self.training_data["training_successful_fraction"] >= self.parameters.rl_parameters.successful_fraction:
            self.save_training_data(save_path_suffix = "FINAL",save_now = True, verbose = True)
            exit()

        #Reset episode specific values
        self.logged_parameters["step_number_in_episode"] = 0
        self.logged_parameters["episode_reward"] = 0
        return


    def on_step(self):
        """Function performs operation that is executed in each timestep. """
        done_numeric = self.training_data["done_numeric"]
        if done_numeric == 1: self.logged_parameters["done_numeric_1"] += 1
        if done_numeric == 2: self.logged_parameters["done_numeric_2"] += 1
        if done_numeric == 3: self.logged_parameters["done_numeric_3"] += 1
        if done_numeric == 4: self.logged_parameters["done_numeric_4"] += 1
        if done_numeric == 5: self.logged_parameters["done_numeric_5"] += 1
        if done_numeric == 6: self.logged_parameters["done_numeric_6"] += 1
        if done_numeric == 8: self.logged_parameters["done_numeric_8"] += 1
        touchdown_on_platform = self.training_data["touchdown_on_platform"]
        if touchdown_on_platform == True:
            self.logged_parameters["done_numeric_7"] += 1
        self.logged_parameters["step_number_in_episode"] += 1
        self.logged_parameters["episode_reward"] += self.training_data["reward"]
        self.logged_parameters["total_step_number"] += 1
        return True


def decay_rate_from_schedule(episode:int,schedule:dict,default_value:float):
    '''
    Function determines the decay rate that has been defined using a dictionary. 
    This dictionary contains values for the start and end timestep, the desired start and end decay rate and the decay type.
    Currently implemented are:
    -exponential decay
    -linear decay (option of no decay is possible, which means constant decay value of zero)
    '''
     #Verify        
    for i in range(len(schedule)):
        assert schedule[i][0] in ["exp","lin"],"The decay type '" + schedule[i][0] + "' specified for decay rate schedule entry " +str(i)+ " is not implemented"
        assert schedule[i][1] < schedule[i][2],"The start and end episode number of decay rate schedule entry "+str(i)+" is wrong. Start episode number needs to be smaller than end episode number"
        assert schedule[i][3] >= schedule[i][4],"The start and end decay rate of decay rate schedule entry "+str(i)+" is wrong. Start decay rate needs to be larger or equal than end decay rate"
        
    for i in range(1,len(schedule)):
        assert schedule[i][1] == schedule[i-1][2], "The episode numbers specifying begin and end of decay rate schedule entry "+str(i-1)+" and "+str(i)+" do not match"

    #Initialize tdecay rate with default value
    decay_rate = default_value

    #Get idx of applicable schedule entry
    idx = None
    for i in range(len(schedule)):
        #The first occurence that the episode value matches an entry's episode range, the idx is found and the loop is left
        if episode >= schedule[i][1] and episode < schedule[i][2]:
            idx = i
            break

    #If no idx could be found, idx of the last schedule entry is chosen and the end decay rate returned
    if idx is None:
        idx = len(schedule)-1
        decay_rate = schedule[idx][4]
    else:
        start_episode = schedule[idx][1]
        end_episode = schedule[idx][2]
        start_exp = schedule[idx][3]
        end_exp = schedule[idx][4]

        if schedule[idx][0] == 'exp':
            B = (np.log(start_exp) - np.log(end_exp))/(start_episode-end_episode)
            A = start_exp/(np.exp(B*start_episode))
            #Exponential decay
            decay_rate = A*np.exp(B*episode)
        elif schedule[idx][0] == 'lin':
            decay_factor = (end_exp - start_exp)/(end_episode - start_episode)
            decay_rate = start_exp + decay_factor*(episode-start_episode)
    return decay_rate
