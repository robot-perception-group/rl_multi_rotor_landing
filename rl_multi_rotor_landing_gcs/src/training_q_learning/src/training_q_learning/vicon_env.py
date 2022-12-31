"""
Script contains the definition of the class defining the training environment and some of its interfaces to other ros nodes.
Furthermore, it registers the landing scenario as an environment in gym.
"""

import gym
import rospy
import numpy as np
import time
from gym import spaces
from geometry_msgs.msg import Vector3
from gym.envs.registration import register
import os
import time
from std_msgs.msg import Float64, Bool
from training_q_learning.msg import Action
from training_q_learning.vicon_object import ViconObject
from training_q_learning.msg import Action
from training_q_learning.parameters import Parameters 
from training_q_learning.utils import get_publisher
from training_q_learning.utils_multiresolution import get_discrete_state_from_ros_msg
from std_msgs.msg import Bool
from copy import deepcopy
import time

#Script variables
node_name = 'vicon_gym_node'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name','iris')


#Topic definitions
action_to_interface_topic = ('command/action',Action)
reset_simulation_topic = ("flight/reset",Bool)
reward_topic =('flight/reward',Float64)
step_execution_frequency_topic = ('flight/step/execution_frequency',Float64)
timestep_error_topic = ('flight/step/time_step_error',Float64)


# Register the training environment in gym as an available one
reg = register(
    id='vicon-v0', 
    entry_point='training_q_learning.vicon_env:ViconEnv',
    )


class ViconEnv(gym.Env):
    def __init__(self):
        '''Class for setting up a gym based training environment that can be used in combination with Gazebo to train an agent to land on a moving platform.'''
        #Get parameters
        self.parameters = Parameters()
        self.drone_name = drone_name
        
        #Set the random number seed used for initializing the drone
        np.random.seed(self.parameters.rl_parameters.seed_init)

        #Create publishers and services
        self.action_to_interface_publisher = get_publisher(action_to_interface_topic[0],action_to_interface_topic[1],queue_size = 0)
        self.reset_simulation_publisher = get_publisher(reset_simulation_topic[0],reset_simulation_topic[1],queue_size = 0)
        self.step_execution_frequency_publisher = get_publisher(step_execution_frequency_topic[0],step_execution_frequency_topic[1],queue_size = 0)
        self.time_step_error_publisher = get_publisher(timestep_error_topic[0],timestep_error_topic[1],queue_size = 0)
        self.reward_publisher = get_publisher(reward_topic[0],reward_topic[1],queue_size = 0)
        
        #Initialize object defining the learning behavior
        self.vicon_object = ViconObject(drone_name)

        #Initialize action space
        self.action_strings = self.parameters.uav_parameters.action_strings
        self.action_values = deepcopy(self.parameters.uav_parameters.initial_action_values)
        
        #Initialize variables required for logging system
        self.reward = 0
        self.episode = 0
        self.step_number_in_episode = 0
        self.episode_reward = 0
        self.reset_happened = False
        self.done_numeric = 0
        self.touchdown_on_platform = False
        self.vicon_object.touchdown_on_platform = False
        self.exploration_rate = 0
        
        #Set step parameters
        self.running_step_time = self.parameters.rl_parameters.running_step_time

        #Set parameters required for initilaization of drone after reset
        self.drone_name = self.parameters.uav_parameters.drone_name
        self.max_x = self.parameters.simulation_parameters.init_max_x
        self.min_x = self.parameters.simulation_parameters.init_min_x
        self.max_y = self.parameters.simulation_parameters.init_max_y
        self.min_y = self.parameters.simulation_parameters.init_min_y
        self.max_z = self.parameters.simulation_parameters.init_max_z
        self.min_z = self.parameters.simulation_parameters.init_min_z
        self.init_altitude = self.parameters.simulation_parameters.init_altitude

        #Other script variables
        # self.test_mode_activated = False
        self.log_dir_path = None
        self.date_prefix = None

        print("Done setting up vicon environment...")
        return

 
    def update_action_values(self,action:int):
        ''' Function maps an action integer number to an action string, updates the new setpoints for the attitude controller of the multi-rotor vehicle and and saves them in the environment class.'''
        cmd = self.parameters.uav_parameters.action_strings[action]
        
        #Decompose the action command to identify operation to be taken
        cmd_name = cmd.split("_")[1] #can be pitch, roll, v_z, yaw or nothing
        cmd_action = cmd.split("_")[0] # can be increase or decrease or do

        #Uodate action values that are send to the attitude controllers 
        if not cmd_name == "nothing":
            if cmd_action == "increase":
                self.action_values[cmd_name] += self.parameters.uav_parameters.action_delta_values[cmd_name]
                if self.action_values[cmd_name] > self.parameters.uav_parameters.action_max_values[cmd_name]:
                    self.action_values[cmd_name] = self.parameters.uav_parameters.action_max_values[cmd_name]

            elif cmd_action == "decrease":
                self.action_values[cmd_name] -= self.parameters.uav_parameters.action_delta_values[cmd_name]
                if self.action_values[cmd_name] < -self.parameters.uav_parameters.action_max_values[cmd_name]:
                    self.action_values[cmd_name] = -self.parameters.uav_parameters.action_max_values[cmd_name]

            else:
                print("Detected not implemented action for "+cmd_name)
                raise ValueError
        else:
            #Action taken was "do_nothing". So nothing is done.
            pass

        #Pass the updated values to the vicon_object
        self.vicon_object.action_values = deepcopy(self.action_values)
        return


    def reset(self): 
        '''Function resets the training environment and updates logging data '''

        #Send reset signal to ROS network
        self.send_reset_simulation_signal()

        #Reset the setpoints for the lowlevel controllers of the copter
        self.action_values = deepcopy(self.parameters.uav_parameters.initial_action_values)
        self.publish_action_to_interface()

        #Extract terminal condition that led to reset
        self.done_numeric = self.vicon_object.done_numeric
        self.touchdown_on_platform = self.vicon_object.touchdown_on_platform

        #Collect the new states
        self.vicon_object.reset_observation = True
        observation_msg = self.vicon_object.get_observation()
        self.vicon_object.reset_observation = False
        observation = self.convert_observation_msg(observation_msg)    

        #Update the parameters required to run the simulation
        self.step_number_in_episode = 0
        self.vicon_object.step_number_in_episode = 0
        self.episode_reward = 0
        self.vicon_object.cum_reward = 0
        self.vicon_object.done_numeric = 0
        self.reset_happened = True
        return observation
        
        
    def send_reset_simulation_signal(self):
        '''Function sends out a boolean value indicating that a reset has been requested. This can be used in other nodes that need reset, such as the action to training interface node. '''
        msg_reset = Bool()
        msg_reset.data = True
        self.reset_simulation_publisher.publish(msg_reset)
        return


    def convert_observation_msg(self, observation_list:list):
        ''' Function performs the normalization of observations of the environment and clips them to a range of [-1,1].'''
        #The observation list contains the continuous value of relative states and the action setpoint, both in ros msgs.
        #Retrieve relative state message
        rel_state  = observation_list[0]

        #Apply conversion to all observations specified to be discretized to form the state space of the RL problem
        for msg_string in self.parameters.uav_parameters.observation_msg_strings.values():
            #Get the max value that is to be used for normalization
            max_value = self.parameters.uav_parameters.observation_max_values[msg_string]

            #Perform the normalization
            n_tmp_value = getattr(rel_state,msg_string)/max_value

            #Clip the values to value range [-1,1]
            n_tmp_value = np.clip(n_tmp_value,-1,1)

            #Reassign values to relative state message
            setattr(rel_state,msg_string,n_tmp_value)

        #Define returned list of modified observations 
        observation = [rel_state,observation_list[1]]

        #Assign to landing_simulation_object
        return observation


    def publish_action_to_interface(self):
        """Function publishes the action values that are currently set to the ROS network."""
        msg_action = Action()
        msg_action.header.stamp = rospy.Time.now()
        for msg_string in self.action_values.keys():
            setattr(msg_action,msg_string,self.action_values[msg_string])    
        self.action_to_interface_publisher.publish(msg_action)
        return


    def render(self):
        """Not implemented."""
        pass  


    def close(self):
        """Not implemented."""
        pass

    def step_2D(self,action_lon,action_lat,parameters_lon,parameters_lat,lims_of_cur_steps_lon,lims_of_cur_steps_lat):
        """Function performs one time step for when two instances of the same agent are used to independently control longitudinal and lateral motion of the multi-rotor vehicle assuming a symmetric vehicle."""
        #Reset touchdown_value if previous step was the reset step
        t_step_start = time.time()
        if self.reset_happened == True:
            self.vicon_object.touchdown_on_platform = False
            self.touchdown_on_platform = False
            self.reset_happened = False
        
        #Provide parameters needed for lon movement
        self.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
        self.update_action_values(action_lon)
        
        #Provide parameters needed for lat movement
        self.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
        self.update_action_values(action_lat)
        
        #Publish data to the ROS network
        self.publish_action_to_interface()
        
        #Let simulation run for one timestep and determine the elapsed time using the wall clock
        t_start_f = time.time()
        rospy.sleep(self.running_step_time)
        t_stop_f = time.time()

        #Get observation
        observation_msg_list = self.vicon_object.get_observation()


        #Process the results of the timestep for longitudinal motion
        self.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
        self.vicon_object.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
        observation_lon = self.convert_observation_msg(observation_msg_list)
        setattr(observation_lon[0],"rel_p_x",-getattr(observation_lon[0],"rel_p_x"))
        setattr(observation_lon[0],"rel_v_x",-getattr(observation_lon[0],"rel_v_x"))
        setattr(observation_lon[0],"rel_a_x",-getattr(observation_lon[0],"rel_a_x"))
        current_state_idx_lon =  get_discrete_state_from_ros_msg(observation_lon[0],observation_lon[1],lims_of_cur_steps_lon,self.vicon_object.n_r,self.parameters)
        self.vicon_object.current_state_idx = deepcopy(current_state_idx_lon)
        (done_lon,_) = self.vicon_object.process_data() 

        #Process the results of the timestep for lateral motion
        self.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
        self.vicon_object.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
        observation_lat = self.convert_observation_msg(observation_msg_list)
        current_state_idx_lat =  get_discrete_state_from_ros_msg(observation_lat[0],observation_lat[1],lims_of_cur_steps_lat,self.vicon_object.n_r,self.parameters)
        self.vicon_object.current_state_idx = deepcopy(current_state_idx_lat)
        (done_lat,_) = self.vicon_object.process_data() 
        
        #Switch back to longitudinal motion (default)
        self.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)

        #Check if a terminal criterion was met
        if done_lon or done_lat:
            done = True
        else:
            done = False

        #Update the episode number
        self.step_number_in_episode += 1
        self.vicon_object.step_number_in_episode = self.step_number_in_episode
        info = {}

        #Publish results of time measurement
        duration_step = t_stop_f-t_start_f
        execution_frequency_msg = Float64()
        execution_frequency_msg.data = 1/duration_step
        self.step_execution_frequency_publisher.publish(execution_frequency_msg)
        time_step_error_msg = Float64()
        time_step_error_msg.data = self.running_step_time-duration_step
        self.time_step_error_publisher.publish(time_step_error_msg)

        t_step_stop = time.time()
        print("Step freq = ",1/(t_step_stop-t_step_start))

        return current_state_idx_lon,current_state_idx_lat, done, info


if __name__ == '__main__':
    #Init nodes and subscribers
    rospy.init_node(node_name)
    landing_simulation_env = ViconEnv()
    landing_simulation_env.reset()
    action = Vector3()
    action.x = 1
    landing_simulation_env.update_action_values(action),
    landing_simulation_env._step()
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        #do nothing
        rate.sleep()
        
        
    
