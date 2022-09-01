#!/usr/bin/env python
'''
Functions needed for running the training.
'''

import rospy
from std_msgs.msg import Float64,Float64MultiArray
from training_q_learning.msg import ObservationRelativeState as ObservationRelativeStateMsg
from training_q_learning.msg import  Action as ActionMsg
from training_q_learning.msg import LandingSimulationObjectState
from training_q_learning.utils import get_publisher
from training_q_learning.parameters import Parameters
import numpy as np
import os
from copy import deepcopy
from training_q_learning.utils import  get_publisher
from training_q_learning.utils_multiresolution import add_to_dict_refinement_min_max_values_for_state


#Parameters
node_name = 'landing_simulation_node'


class ViconObject():
    ''' Class contains all parameters and functions necessary to run the landing simulation in a training environment.
    The functions comprise getting the state, reward computation, checking if the episode is over, checking if the reset was successfull,
    translation of action into required values for ROS / Gazebo values.
    '''
    def __init__(self,drone_name,ros_port,gaz_port,ros_ip):#Add the simulation parameters here
        '''
        Class contains information about the training such as reward computation, handling of terminal events and reading observations of the environment.
        '''
        # Initialize parameters
        self.parameters = Parameters()
        self.drone_name = drone_name
        self.topic_prefix = '/'+self.drone_name+'/'

        # Set up environment to run multiple simulations in parallel
        print("landing_simulation_object: ros_port = ",ros_port)
        print("landing_simulation_object: gaz_port = ",gaz_port)
        print("landing_simulation_object: ros_ip = ",ros_ip)
        host_addr = "http://10.42.0.105:"
        os.environ["ROS_MASTER_URI"] = host_addr + str(ros_port) + "/"
        os.environ["GAZEBO_MASTER_URI"] = host_addr + str(gaz_port) + "/"
        
        #Set up subscribers
        self.observation_continous_subscriber = rospy.Subscriber(self.topic_prefix+'vicon_observation_interface/observations',ObservationRelativeStateMsg,self.read_training_continous_observations)
        self.observation_drone_state_subscriber = rospy.Subscriber(self.topic_prefix+'/vicon/drone/state',LandingSimulationObjectState,self.read_drone_state)

        #Set up publisher
        self.relative_reward_publisher = get_publisher(self.topic_prefix+'flight/relative_change_reward',Float64,queue_size = 0,ros_ip = ros_ip,ros_port = ros_port,gaz_port = gaz_port)
        self.reward_current_state_publisher = get_publisher(self.topic_prefix+'flight/current_state_reward',Float64,queue_size = 0,ros_ip = ros_ip,ros_port = ros_port,gaz_port = gaz_port)
        self.cum_reward_publisher = get_publisher(self.topic_prefix+'flight/episode_cum_reward',Float64,queue_size = 0,ros_ip = ros_ip,ros_port = ros_port,gaz_port = gaz_port)
        self.reward_rel_pos_publisher = get_publisher(self.topic_prefix+'flight/reward_rel_pos',Float64,queue_size = 0,ros_ip = ros_ip,ros_port = ros_port,gaz_port = gaz_port)
        self.reward_rel_vel_publisher = get_publisher(self.topic_prefix+'flight/reward_rel_vel',Float64,queue_size = 0,ros_ip = ros_ip,ros_port = ros_port,gaz_port = gaz_port)
        self.reward_action_publisher = get_publisher(self.topic_prefix+'flight/reward_action',Float64,queue_size = 0,ros_ip = ros_ip,ros_port = ros_port,gaz_port = gaz_port)
        self.current_idx_publisher = get_publisher(self.topic_prefix+'flight/current_idx',Float64MultiArray,queue_size = 0,ros_ip = ros_ip,ros_port = ros_port,gaz_port = gaz_port)
        
        #Initialize observation variables
        self.reset_observation = False
        self.observation_continous = ObservationRelativeStateMsg()
        self.observation_continous_actions = ActionMsg()
        self.observation_drone_state = LandingSimulationObjectState()

        #Initialize the dictionaries containing information about the current discretization
        #Will be updated with actual values during training
        refinement_steps_dict = dict()  #Dictionary containing the (normalized) positive and negative limit values for each observation specified in the parameters 
        for msg_string in self.parameters.uav_parameters.observation_msg_strings.values():
            add_to_dict_refinement_min_max_values_for_state(msg_string,refinement_steps_dict)   
        self.current_ref_step_counter = 0
        self.refinement_steps_dict = deepcopy(refinement_steps_dict)

        #Set up the current state idx
        self.current_state_idx = np.zeros(4,dtype = int) #Integer array containing the idx of the state in the current refinement
        self.previous_state_idx = deepcopy(self.current_state_idx) #Integer array containing the idx of the state in the previous timestep
        self.previous_shaping_dict = dict()  #Dictionary containing the shaping values for each observation of the previous timestep
        for i in range(len(refinement_steps_dict["rel_p_x"])): self.previous_shaping_dict[i] = 0
        self.current_shaping_dict = deepcopy(self.previous_shaping_dict)

        #Initialize the arrays containing the shaping value of the current timestep for each refinement 
        self.current_shaping_value_pos = np.zeros(len(self.current_shaping_dict))
        self.current_shaping_value_vel = np.zeros(len(self.current_shaping_dict))
        self.current_shaping_value_action = np.zeros(len(self.current_shaping_dict))
        self.previous_shaping_value_pos = np.zeros(len(self.current_shaping_dict))
        self.previous_shaping_value_vel = np.zeros(len(self.current_shaping_dict))
        self.previous_shaping_value_action = np.zeros(len(self.current_shaping_dict))

        #Episode completion variables
        self.done = 0
        self.touchdown_on_platform = False
        self.step_number_in_episode = 0
       
        #Other variables required for execution
        self.cum_reward = 0
        self.action_values = deepcopy(self.parameters.uav_parameters.initial_action_values) #Dictionary in which the current set points for the roll pitch yawrate thrust controller are stored
        self.done_numeric = 0
        self.max_number_of_steps_in_episode = self.parameters.rl_parameters.max_num_timesteps_episode
        self.touch_down_height = self.parameters.simulation_parameters.touch_down_height
        self.test_mode_activated = False
        return
  
    def publish_current_idx(self):
        '''Publication of the current discrete state to the ROS environment.'''
        msg = Float64MultiArray()
        msg.data = self.current_state_idx
        self.current_idx_publisher.publish(msg)
        return

    def read_drone_state(self,msg):
        '''Function reads the current state of the drone whenever triggered by the corresponding subscriber to the corresponding ROS topic.'''
        self.observation_drone_state = msg
        return


    def read_training_continous_observations(self,msg):
        '''Functions reads the continouos observations of the environment whenever the corresponding subsriber to the corresponding ROS topic is triggered.'''
        self.observation_continous = msg
        return


    def check_done_criteria(self):
        ''' Function checks whether or not the episode has to be terminated or not. It can decide between successful termination, unsuccessful termintation
        and episode not yet finished.
        '''
        n_r = self.parameters.rl_parameters.n_r

        #Determine if discrete goal state has been reached
        goal_state_reached = False

        if self.parameters.rl_parameters.n_r % 2 == 0:
            #even number of intervals
            pass
            print("Implementation of even number of intervals needs to be checked. Aborting...")
            exit()
        else:
            #uneven number
            if len(self.parameters.uav_parameters.action_max_values) == 1:
                if self.current_state_idx[0] == self.previous_state_idx[0]: 
                    self.current_ref_step_counter += 1
                    if self.current_ref_step_counter > int(self.parameters.rl_parameters.ref_step_success_duration) and self.current_state_idx[0] == len(self.refinement_steps_dict["rel_p_x"]) - 1 and self.current_state_idx[1] == (n_r-1)/2 and self.current_state_idx[2]  == (n_r-1)/2:
                        goal_state_reached = True
                        print("Uneven n_r(",n_r,"), state space rel_pos, rel_vel")
                else:
                    self.current_ref_step_counter = 0
            elif len(self.parameters.uav_parameters.action_max_values) == 2:
                print("Implementation of more than one action value needs to be checked first. Aborting...")
                exit()

        #Determine the reason why an episode needs to be ended
        done_numeric = 0
        if self.step_number_in_episode >= self.max_number_of_steps_in_episode:
            #Not successful termination
            done_numeric = 1
            self.current_ref_step_counter = 0
        elif self.parameters.simulation_parameters.position_terminal_mode == 'relative' and abs(self.observation_continous.rel_p_x) >= self.parameters.simulation_parameters.max_abs_p_x:
            done_numeric = 3
            self.current_ref_step_counter = 0
        elif self.parameters.simulation_parameters.position_terminal_mode == 'absolute' and abs(self.observation_drone_state.pose.pose.position.x) >= self.parameters.simulation_parameters.max_abs_p_x:
            done_numeric = 3
            self.current_ref_step_counter = 0
        elif self.parameters.simulation_parameters.position_terminal_mode == 'relative' and abs(self.observation_continous.rel_p_y) >= self.parameters.simulation_parameters.max_abs_p_y:
            done_numeric = 4
            self.current_ref_step_counter = 0
        elif self.parameters.simulation_parameters.position_terminal_mode == 'absolute' and abs(self.observation_drone_state.pose.pose.position.y) >= self.parameters.simulation_parameters.max_abs_p_y:
            done_numeric = 4
            self.current_ref_step_counter = 0
        elif self.observation_continous.rel_p_z < self.touch_down_height:
            done_numeric = 7
            self.current_ref_step_counter = 0
        elif goal_state_reached ==  True:
            done_numeric = 8
            self.current_ref_step_counter = 0
        else:
            done_numeric = 0
        self.done_numeric = done_numeric
        return done_numeric

    

    def get_observation(self):
        '''Function reads observation from corresponding topic and updates old observation values.
        In its current implementation, it outputs the continouos observation.
        '''
        #Create ros msg for setpoint action values
        action_msg = ActionMsg()
        for msg_string in self.action_values.keys():
            setattr(action_msg,msg_string,self.action_values[msg_string])
        return [deepcopy(self.observation_continous),action_msg]


    def compute_reward(self):
        '''
        Function computes the reward function of the agent for the current timestep.
        '''
        self.publish_current_idx()

        #Get the continuous values of the observations rel_p_x and rel_v_x
        c_rel_p_x = self.observation_continous.rel_p_x 
        c_rel_v_x = self.observation_continous.rel_v_x 

        #Conpute normalized and clipped values
        n_c_rel_p_x = np.clip(c_rel_p_x / self.parameters.uav_parameters.observation_max_values["rel_p_x"],-1,1) #normalized
        n_c_rel_v_x = np.clip(c_rel_v_x / self.parameters.uav_parameters.observation_max_values["rel_v_x"],-1,1) #normalized

        #Add variable for potential scaling --> currently no scaling applied
        n_c_scaled_rel_p_x = n_c_rel_p_x/1
        n_c_scaled_rel_v_x = n_c_rel_v_x/1

        #Get normalized current action value
        n_action_pitch = self.action_values["pitch"] / self.parameters.uav_parameters.action_max_values["pitch"]

        #Get the current refinement step / curriculum step (synymously used here)
        ref_step_idx = int(self.current_state_idx[0])

        #Get the weights defining the gradient of the shaping function
        grad_start_rel_p = self.parameters.simulation_parameters.grad_start_rel_p
        grad_start_rel_v = self.parameters.simulation_parameters.grad_start_rel_v

        #Get the action weight
        action_weight = self.parameters.simulation_parameters.action_weight

        #Set up the messages for later publishing
        reward_rel_pos_msg = Float64()
        reward_rel_vel_msg = Float64()
        reward_rel_action_msg = Float64()
        msg_cum_reward = Float64()
        reward_rel_pos_msg.data = 0
        reward_rel_vel_msg.data = 0
        reward_rel_action_msg.data = 0
        msg_cum_reward.data = 0

        #Iterate through all refinement steps / curriculum steps that are currently available and determine the current timesteps shaping values
        for i in range(len(self.current_shaping_dict)):
            self.current_shaping_value_pos[i] = grad_start_rel_p*np.abs(n_c_scaled_rel_p_x)
            self.current_shaping_value_vel[i] = grad_start_rel_v*np.abs(n_c_scaled_rel_v_x)
            self.current_shaping_value_action[i] = action_weight*np.abs(n_action_pitch)
        
        #Compute reward value
        reward_duration = self.refinement_steps_dict['rel_v_x'][ref_step_idx][1]*self.parameters.simulation_parameters.reward_duration

        #Positional contribution to total reward
        rel_pos_reward_max_abs = abs( self.parameters.simulation_parameters.grad_start_rel_p) * self.refinement_steps_dict['rel_v_x'][ref_step_idx][1] * self.parameters.rl_parameters.running_step_time
        reward_rel_pos = np.clip(self.current_shaping_value_pos[ref_step_idx] - self.previous_shaping_value_pos[ref_step_idx],-rel_pos_reward_max_abs,rel_pos_reward_max_abs)
        
        #Velocity contribution to total reward
        rel_vel_reward_max_abs = abs(self.parameters.simulation_parameters.grad_start_rel_v)*self.refinement_steps_dict['rel_a_x'][ref_step_idx][1] * self.parameters.rl_parameters.running_step_time
        reward_rel_vel = np.clip(self.current_shaping_value_vel[ref_step_idx] - self.previous_shaping_value_vel[ref_step_idx],-rel_vel_reward_max_abs,rel_vel_reward_max_abs)

        #Action contribution to total reward
        reward_rel_action = self.refinement_steps_dict['rel_v_x'][ref_step_idx][1]*(self.current_shaping_value_action[ref_step_idx] - self.previous_shaping_value_action[ref_step_idx])
        rel_action_reward_max_abs = self.refinement_steps_dict['rel_v_x'][ref_step_idx][1]*action_weight*(self.parameters.uav_parameters.action_delta_values["pitch"]/self.parameters.uav_parameters.action_max_values["pitch"])
        
        #Determine the maximum possible reward available in one timestep while being in the current refinement step / curriculum step
        self.max_possible_reward_for_one_timestep = rel_pos_reward_max_abs + rel_vel_reward_max_abs + rel_action_reward_max_abs + reward_duration
        
        reward_ref_step = 0
        if self.current_state_idx[0]<self.previous_state_idx[0]:
            #if the curriculum step changed to a previous one
            reward_ref_step = self.parameters.simulation_parameters.reward_ref_step_decrease*self.max_possible_reward_for_one_timestep

        if self.current_state_idx[0]>self.previous_state_idx[0]:
            #If the curriculum step has changed to a later one.
            reward_ref_step = self.parameters.simulation_parameters.reward_success*self.max_possible_reward_for_one_timestep

        #Compute the total reward
        reward_total = reward_rel_pos + reward_rel_vel + reward_rel_action + reward_ref_step + reward_duration
        
        #Update cummulative reward value
        self.cum_reward += reward_total
        
        #Publish the different rewards and the different contributions to it
        reward_rel_pos_msg.data = reward_rel_pos
        reward_rel_vel_msg.data = reward_rel_vel
        reward_rel_action_msg.data = reward_rel_action
        msg_cum_reward.data = self.cum_reward
        self.reward_rel_pos_publisher.publish(reward_rel_pos_msg)
        self.reward_rel_vel_publisher.publish(reward_rel_vel_msg)
        self.reward_action_publisher.publish(reward_rel_action_msg)
        self.cum_reward_publisher.publish(msg_cum_reward)

        #Store the current values for the next timestep
        self.previous_shaping_dict = deepcopy(self.current_shaping_dict)
        self.previous_state_idx = deepcopy(self.current_state_idx)
        self.previous_shaping_value_pos = deepcopy(self.current_shaping_value_pos)
        self.previous_shaping_value_vel = deepcopy(self.current_shaping_value_vel)
        self.previous_shaping_value_action = deepcopy(self.current_shaping_value_action)      
        return reward_total

    def process_data(self):
        """Function checks episode termination status and generates the appropriate reward."""

        #Compute different reward values
        # reward_leave_fly_zone = self.parameters.simulation_parameters.reward_leave_fly_zone*self.max_possible_reward_for_one_timestep
        # reward_success = self.parameters.simulation_parameters.reward_success*self.max_possible_reward_for_one_timestep
        # reward_max_timestep_in_episode = self.parameters.simulation_parameters.max_timestep_in_episode*self.max_possible_reward_for_one_timestep

        #Init terminal criteria checking
        done = False
        done_numeric = self.check_done_criteria()
        reward = 0

        if done_numeric == 0:
            if not self.test_mode_activated:
                reward = self.compute_reward()
            else:
                reward = 0
        elif done_numeric == 1:
            print('END OF EPISODE: Max. number of steps in episode reached...')
            # reward = reward_max_timestep_in_episode
            done = self.parameters.simulation_parameters.done_criteria["max_num_timesteps"]
        elif done_numeric == 2:
            print('END OF EPISODE: Vertical distance between drone and moving platform too big...')
            # reward = reward_leave_fly_zone 
            done = self.parameters.simulation_parameters.done_criteria["max_ver_distance"]
        elif done_numeric == 3:
            print('END OF EPISODE: Longitudinal distance between drone and moving platform too big...')
            # reward = reward_leave_fly_zone 
            done = self.parameters.simulation_parameters.done_criteria["max_lon_distance"]
        elif done_numeric == 4:
            print('END OF EPISODE: Latteral distance between drone and moving platform too big...')
            # reward = reward_leave_fly_zone 
            done = self.parameters.simulation_parameters.done_criteria["max_lat_distance"]       
        elif done_numeric == 7: 
            print('END OF EPISODE: Touchdown height reached...')
            # reward = 0
            done = self.parameters.simulation_parameters.done_criteria["touchdown"]
        elif done_numeric == 8:
            done = self.parameters.simulation_parameters.done_criteria["success"]
            # reward = reward_success
            print('SUCCESS: GOAL STATE REACHED')          
        return done, reward
        
if __name__ == '__main__':
    #Init nodes and subscribers
    rospy.init_node(node_name)
    landing_simulation_object = ViconObject()
    
