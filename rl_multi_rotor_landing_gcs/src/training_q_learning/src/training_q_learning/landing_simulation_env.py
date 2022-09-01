#!/usr/bin/env python
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
from std_srvs.srv import Empty
from training_q_learning.msg import Action
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from datetime import datetime
from training_q_learning.landing_simulation_object import LandingSimulationObject
from training_q_learning.msg import Action
from training_q_learning.parameters import Parameters 
from training_q_learning.utils import get_publisher, write_data_to_csv
from training_q_learning.utils_multiresolution import get_state_grid_idx_from_ros_msg
from std_msgs.msg import Bool
from copy import deepcopy

#Script variables
node_name = 'landing_simulation_gym_node'
ros_port = rospy.get_param(rospy.get_namespace()+node_name+'/ros_port',0)
gaz_port = rospy.get_param(rospy.get_namespace()+node_name+'/gaz_port',0)
ros_ip = rospy.get_param(rospy.get_namespace()+node_name+'/ros_ip',0)
M_PI = 3.1415926535

#Topic definitions
action_to_interface_topic = ('training_action_interface/action_to_interface',Action)
reset_simulation_topic = ("training/reset_simulation",Bool)
reward_topic =('training/reward',Float64)
step_execution_frequency_topic = ('training/step/execution_frequency',Float64)
timestep_error_topic = ('training/step/time_step_error',Float64)




# Register the training environment in gym as an available one
reg = register(
    id='landing_simulation-v0', 
    entry_point='training_q_learning.landing_simulation_env:LandingSimulationEnv',
    )


class LandingSimulationEnv(gym.Env):
    def __init__(self):
        '''Class for setting up a gym based training environment that can be used in combination with Gazebo to train an agent to land on a moving platform.'''
        #Get parameters
        self.parameters = Parameters()
        
        #Set the random number seed used for initializing the drone
        np.random.seed(self.parameters.rl_parameters.seed_init)

        #Set up script to enable running of multiple simulations in parallel
        print("landing_simulation_env: ros_port = ",ros_port)
        print("landing_simulation_env: gaz_port = ",gaz_port)
        print("landing_simulation_env: ros_ip = ",ros_ip)
        host_addr = "http://" + str(ros_ip) + ":"
        os.environ["ROS_MASTER_URI"] = host_addr + str(ros_port) + "/"
        os.environ["GAZEBO_MASTER_URI"] = host_addr + str(gaz_port) + "/"

        #Create numpy array for duration measurement to find out how long it takes to get observation
        self.get_observation_durations = np.zeros(int(1/self.parameters.rl_parameters.running_step_time))
        #Create publishers and services
        self.action_to_interface_publisher = get_publisher(action_to_interface_topic[0],action_to_interface_topic[1],queue_size = 0,ros_ip = ros_ip,ros_port = ros_port,gaz_port = gaz_port)
        self.reset_simulation_publisher = get_publisher(reset_simulation_topic[0],reset_simulation_topic[1],queue_size = 0,ros_ip = ros_ip,ros_port = ros_port,gaz_port = gaz_port)
        self.step_execution_frequency_publisher = get_publisher(step_execution_frequency_topic[0],step_execution_frequency_topic[1],queue_size = 0,ros_ip = ros_ip,ros_port = ros_port,gaz_port = gaz_port)
        self.time_step_error_publisher = get_publisher(timestep_error_topic[0],timestep_error_topic[1],queue_size = 0,ros_ip = ros_ip,ros_port = ros_port,gaz_port = gaz_port)
        self.reward_publisher = get_publisher(reward_topic[0],reward_topic[1],queue_size = 0,ros_ip = ros_ip,ros_port = ros_port,gaz_port = gaz_port)
        rospy.wait_for_service('/gazebo/reset_world')
        self.reset_world_gazebo_service = rospy.ServiceProxy('/gazebo/reset_world',Empty)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        rospy.wait_for_service('/gazebo/pause_physics')
        self.pause_sim = rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause_sim = rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        rospy.wait_for_service('/gazebo/get_model_state')
        self.model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
    
        #Initialize object defining the learning behavior
        self.landing_simulation_object = LandingSimulationObject(self.parameters.uav_parameters.drone_name,ros_port,gaz_port,ros_ip)
        #Initialize observation space
        dim_observation_space = len(self.parameters.uav_parameters.observation_msg_strings)
        self.observation_space = spaces.Box(low=np.full((dim_observation_space),-1), high=np.full((dim_observation_space),1), dtype=np.float32)

        #Initialize action space
        print("len(action_strings) = ",len(self.parameters.uav_parameters.action_strings))
        self.action_space = spaces.Discrete(len(self.parameters.uav_parameters.action_strings))
        self.action_strings = self.parameters.uav_parameters.action_strings
        self.action_values = deepcopy(self.parameters.uav_parameters.initial_action_values)
        self.action_string = ''
        
        #Initialize variables required for logging system
        self.reward = 0
        self.episode = 0
        self.step_number_in_episode = 0
        self.episode_reward = 0
        self.reset_happened = False
        self.done_numeric = 0
        self.touchdown_on_platform = False
        self.landing_simulation_object.touchdown_on_platform = False
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
        self.init_height = self.parameters.simulation_parameters.init_height

        #Other script variables
        self.test_mode_activated = False
        self.log_dir_path = None
        self.date_prefix = None

        print("Done setting up training environment...")
        return

 
    def _update_action_values(self,action):
        ''' Function maps action integer number to action string, updates the new setpoints for the attitude controller of the multi-rotor vehicle and and saves them in the environment class.'''
        msg_string = self.parameters.uav_parameters.action_strings[action]
        self.action_string = msg_string
        
        cmd_name = msg_string.split("_")[1] #can be pitch, roll, v_z, yaw
        cmd_action = msg_string.split("_")[0] # can be increase or decrease

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
                print("Detected unspecified action for "+cmd_name)
                raise ValueError
        else:
            #Action taken was "do_nothing". So nothing is done.
            pass
        #Pass the updated values to the landing_simulation_object
        self.landing_simulation_object.action_values = deepcopy(self.action_values)
        return


    def reset(self): 
        '''Function resets the training environment and updates logging data '''
        
        #Send reset signal to ROS network
        self._send_reset_simulation_signal()

        #Reset the setpoints for the lowlevel controllers of the copter
        self.action_values = deepcopy(self.parameters.uav_parameters.initial_action_values)
        self.publish_action_to_interface()

        #Give time to send and process reset signal in the analysis node
        if self.test_mode_activated == True:
            self.unpause_sim()
            rospy.sleep(1.)
            self.pause_sim()

        #Extract terminal condition that led to reset
        self.done_numeric = self.landing_simulation_object.done_numeric
        self.touchdown_on_platform = self.landing_simulation_object.touchdown_on_platform

        #Init the reset process
        #Get platform coordinates before reset
        object_coordinates_moving_platform = self.model_coordinates("moving_platform","world")
        self.pause_sim()

        #Reset gazebo
        self.reset_world_gazebo_service()

        #Begin computation of new init state of the drone
        init_drone = ModelState()
        init_drone.model_name = self.drone_name

        #Determine the init value according to a probability distribution
        if len(self.parameters.uav_parameters.action_max_values)  == 1:
            if self.parameters.simulation_parameters.init_distribution == 'uniform':
                x_vec = [np.random.uniform(low=self.min_x[0], high=self.min_x[1], size=None),np.random.uniform(low=self.max_x[0], high=self.max_x[1], size=None)]
                y_vec = [np.random.uniform(low=self.min_y[0], high=self.min_y[1], size=None),np.random.uniform(low=self.max_y[0], high=self.max_y[1], size=None)]
                x_init = np.random.choice(x_vec,1)
                y_init = np.random.choice(y_vec,1)
            elif self.parameters.simulation_parameters.init_distribution == 'normal':
                init_mu_x = self.parameters.simulation_parameters.init_mu_x 
                init_sigma_x = self.parameters.simulation_parameters.init_sigma_x
                init_mu_y = self.parameters.simulation_parameters.init_mu_y 
                init_sigma_y = self.parameters.simulation_parameters.init_sigma_y
                
                #Two times the same value in the x_Vec and y_vec, meaning that it is not possible to set limits to ranges
                x_init = np.random.normal(init_mu_x,init_sigma_x)
                y_init = np.random.normal(init_mu_y,init_sigma_y)   
        else:
            print("Only 1D case is implemented. Aborting...")
            exit()

        #Compute the init position within the specified fly zone ('absolute') or relative to the moving platform ('relative')
        if self.parameters.simulation_parameters.init_mode == 'relative':
            init_drone.pose.position.x = np.clip(x_init + object_coordinates_moving_platform.pose.position.x,-self.parameters.simulation_parameters.max_abs_p_x,self.parameters.simulation_parameters.max_abs_p_y)
            init_drone.pose.position.y = np.clip(y_init + object_coordinates_moving_platform.pose.position.y,-self.parameters.simulation_parameters.max_abs_p_y,self.parameters.simulation_parameters.max_abs_p_y)
        elif self.parameters.simulation_parameters.init_mode == 'absolute':
            init_drone.pose.position.x = np.clip(x_init,-self.parameters.simulation_parameters.max_abs_p_x,self.parameters.simulation_parameters.max_abs_p_x) 
            init_drone.pose.position.y = np.clip(y_init,-self.parameters.simulation_parameters.max_abs_p_y,self.parameters.simulation_parameters.max_abs_p_y) 

        #Define the new init state
        init_drone.pose.position.z = self.init_height
        init_drone.twist.linear.x = 0
        init_drone.twist.linear.y = 0
        init_drone.twist.linear.z = 0
        init_drone.twist.angular.x = 0
        init_drone.twist.angular.y = 0
        init_drone.twist.angular.z = 0
        self.set_model_state_service(init_drone)
        
        #Let simulation run for a short time to collect the new values after the initialization
        self.unpause_sim()
        rospy.sleep(0.1)
        object_coordinates_moving_platform_after_reset = self.model_coordinates("moving_platform","world")
        object_coordinates_drone_after_reset = self.model_coordinates(self.parameters.uav_parameters.drone_name,"world")
        self.pause_sim()
        
        #Collect the new states
        self.landing_simulation_object.reset_observation = True
        observation_msg = self.landing_simulation_object.get_observation()
        self.landing_simulation_object.reset_observation = False
        observation = self._convert_observation_msg(observation_msg)

        #Execute reward function once to update the time dependent components in the reward function
        if self.test_mode_activated == False:
            _ = self.landing_simulation_object.compute_reward()

        #Store the values if a log dir path has been set 
        #data to be stored
        t = datetime.now()
        time = [t.strftime("%Y%m%d_%H%M%S")]
        episode_entry = [self.episode]
        moving_platform_position = [object_coordinates_moving_platform_after_reset.pose.position.x,object_coordinates_moving_platform_after_reset.pose.position.y,object_coordinates_moving_platform_after_reset.pose.position.z]
        moving_platform_twist_linear = [object_coordinates_moving_platform_after_reset.twist.linear.x,object_coordinates_moving_platform_after_reset.twist.linear.y,object_coordinates_moving_platform_after_reset.twist.linear.z]
        moving_platform_orientation = [object_coordinates_moving_platform_after_reset.pose.orientation.x,object_coordinates_moving_platform_after_reset.pose.orientation.y,object_coordinates_moving_platform_after_reset.pose.orientation.z,object_coordinates_moving_platform_after_reset.pose.orientation.w]
        drone_position = [object_coordinates_drone_after_reset.pose.position.x,object_coordinates_drone_after_reset.pose.position.y,object_coordinates_drone_after_reset.pose.position.z]
        drone_twist_linear = [object_coordinates_drone_after_reset.twist.linear.x,object_coordinates_drone_after_reset.twist.linear.y,object_coordinates_drone_after_reset.twist.linear.z]
        drone_orientation = [object_coordinates_drone_after_reset.pose.orientation.x,object_coordinates_drone_after_reset.pose.orientation.y,object_coordinates_drone_after_reset.pose.orientation.z,object_coordinates_drone_after_reset.pose.orientation.w]
        row = time+episode_entry + moving_platform_position + moving_platform_twist_linear + moving_platform_orientation + drone_position + drone_twist_linear + drone_orientation

        #Define location of storage
        if self.log_dir_path and self.parameters.rl_parameters.store_logged_init_values_at == 'automatic':
            #Store where also the other training data is stored
            now = datetime.now()
            if not self.date_prefix:
                self.date_prefix = now.strftime("%Y_%m_%d_%H_%M_%S")
            write_data_to_csv(os.path.join(self.log_dir_path,self.date_prefix+"_init_data.csv"),row)
        elif self.parameters.rl_parameters.store_logged_init_values_at and not self.parameters.rl_parameters.store_logged_init_values_at == 'automatic':
            #Store at specified path
            write_data_to_csv(os.path.join(self.parameters.rl_parameters.store_logged_init_values_at +"init_data.csv"),row)
        #If the path is None / False, then init data is not stored

        #Update the parameters required to run the simulation
        self.episode += 1
        self.step_number_in_episode = 0
        self.landing_simulation_object.step_number_in_episode = 0
        self.episode_reward = 0
        self.landing_simulation_object.cum_reward = 0
        self.landing_simulation_object.done_numeric = 0
        self.reset_happened = True

        return observation
        
        
    def _send_reset_simulation_signal(self):
        '''Function sends out a boolean value indicating that a reset has been requested. This can be used in other nodes that need reset, such as the action to training interface node. '''
        msg_reset = Bool()
        msg_reset.data = True
        self.reset_simulation_publisher.publish(msg_reset)
        return


    def _convert_observation_msg(self, observation_list):
        ''' Function performs the normalization of observations of the environment and clips them to a range of [-1,1].'''
        rel_state  = observation_list[0]
        for msg_string in self.parameters.uav_parameters.observation_msg_strings.values():
            max_value = self.parameters.uav_parameters.observation_max_values[msg_string]
            n_tmp_value = getattr(rel_state,msg_string)/max_value
            n_tmp_value = np.clip(n_tmp_value,-1,1)
            setattr(rel_state,msg_string,n_tmp_value)
        observation = [rel_state,observation_list[1]]
        return observation


    def publish_action_to_interface(self):
        """Function publishes the action values that are currently set to the ROS network."""
        msg_action = Action()
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


    def step(self,action):
        """Function performs one timestep of the training."""
        #Reset values if previous step was the reset step
        if self.reset_happened == True:
            self.landing_simulation_object.touchdown_on_platform = False
            self.touchdown_on_platform = False
            self.reset_happened = False

        #Update the setpoints based on the current action and publish them to the ROS network
        self._update_action_values(action)        
        self.publish_action_to_interface()
        
        #Let simulation run for one simulation timestep and measure the wall clock time that has passed during that timestep
        self.unpause_sim()
        t_start_f = time.time()
        rospy.sleep(self.running_step_time)
        t_stop_f = time.time()
        self.pause_sim()
        #Determine how long it takes to get one observation of the environment
        t_start_get_observation = time.time()
        observation_msg_list = self.landing_simulation_object.get_observation()
        t_stop_get_observation = time.time()

        #Determine how long that one timestep took w.r.t. wall clock time
        duration_step = t_stop_f-t_start_f
        #Determine how long it took to get the observation
        duration_get_observation = t_stop_get_observation-t_start_get_observation
       
        #Normalize and clip the observations
        observation = self._convert_observation_msg(observation_msg_list)
        #Map the current observation to a discrete state value
        self.landing_simulation_object.current_state_idx = get_state_grid_idx_from_ros_msg(observation[0],observation[1],self.landing_simulation_object.refinement_steps_dict,self.landing_simulation_object.n_r,self.parameters)

        #Update the number of episodes
        self.step_number_in_episode += 1
        self.landing_simulation_object.step_number_in_episode = self.step_number_in_episode
        #Check if terminal condition is reached
        (done,reward) = self.landing_simulation_object.process_data() 

        info = {}
        self.reward = reward
        self.episode_reward += reward

        #Publish the reward
        msg_reward = Float64()
        msg_reward.data = reward
        self.reward_publisher.publish(msg_reward)

        #Publish the results of the run time measurements. Data can be used to check how much the sim time deviates from the real time when a real time factor of 1 is set in Gazebo.
        execution_frequency_msg = Float64()
        execution_frequency_msg.data = 1/duration_step
        self.step_execution_frequency_publisher.publish(execution_frequency_msg)
        time_step_error_msg = Float64()
        time_step_error_msg.data = self.running_step_time-duration_step
        self.time_step_error_publisher.publish(time_step_error_msg)
        self.get_observation_durations = np.append(self.get_observation_durations[1:],duration_get_observation)
        return observation, reward, done, info


    def step_2D(self,action_lon,action_lat,parameters_lon,parameters_lat,refinement_steps_dict_lon,refinement_steps_dict_lat):
        """Function performs one time step for when two instances of the same agent are used to independently control longitudinal and lateral motion of the multi-rotor vehicle assuming a symmetric vehicle."""
        #Reset touchdown_value if previous step was the reset step
        if self.reset_happened == True:
            self.landing_simulation_object.touchdown_on_platform = False
            self.touchdown_on_platform = False
            self.reset_happened = False
        
        #Provide parameters needed for lon movement
        self.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
        self._update_action_values(action_lon)
        
        #Provide parameters needed for lat movement
        self.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
        self._update_action_values(action_lat)
        
        #Publish data to the ROS network
        self.publish_action_to_interface()
        
        #Let simulation run for one timestep and determine the elapsed time using the wall clock
        self.unpause_sim()
        t_start_f = time.time()
        rospy.sleep(self.running_step_time)
        t_stop_f = time.time()
        self.pause_sim()

        #Determine how long it takes to get one observation w.r.t. wall clock time
        t_start_get_observation = time.time()
        observation_msg_list = self.landing_simulation_object.get_observation()
        t_stop_get_observation = time.time()

        #Determine how long that one timestep took w.r.t. the wall clock time
        duration_step = t_stop_f-t_start_f
        #Determine how long it took to get the observation w.r.t. the wall clock time
        duration_get_observation = t_stop_get_observation-t_start_get_observation
        
        #print("t_stop_get_observation-t_start_get_observation = ", t_stop_get_observation-t_start_get_observation)
        self.get_observation_durations = np.append(self.get_observation_durations[1:],duration_get_observation)

        #Process the results of the timestep for longitudinal motion
        self.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
        self.landing_simulation_object.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
        observation_lon = self._convert_observation_msg(observation_msg_list)
        current_state_idx_lon =  get_state_grid_idx_from_ros_msg(observation_lon[0],observation_lon[1],refinement_steps_dict_lon,self.landing_simulation_object.n_r,self.parameters)
        self.landing_simulation_object.current_state_idx = deepcopy(current_state_idx_lon)
        (done_lon,_) = self.landing_simulation_object.process_data() 

        #Process the results of the timestep for lateral motion
        self.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
        self.landing_simulation_object.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
        observation_lat = self._convert_observation_msg(observation_msg_list)
        setattr(observation_lat[0],"rel_p_y",-getattr(observation_lat[0],"rel_p_y"))
        setattr(observation_lat[0],"rel_v_y",-getattr(observation_lat[0],"rel_v_y"))
        setattr(observation_lat[0],"rel_a_y",-getattr(observation_lat[0],"rel_a_y"))
        current_state_idx_lat =  get_state_grid_idx_from_ros_msg(observation_lat[0],observation_lat[1],refinement_steps_dict_lat,self.landing_simulation_object.n_r,self.parameters)
        self.landing_simulation_object.current_state_idx = deepcopy(current_state_idx_lat)
        (done_lat,_) = self.landing_simulation_object.process_data() 
        
        #Switch back to longitudinal motion (default)
        self.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)

        #Check if a terminal criterion was met
        if done_lon or done_lat:
            done = True
        else:
            done = False

        #Update the episode number
        self.step_number_in_episode += 1
        self.landing_simulation_object.step_number_in_episode =self.step_number_in_episode
        info = {}

        #Publish results of time measurement
        execution_frequency_msg = Float64()
        execution_frequency_msg.data = 1/duration_step
        self.step_execution_frequency_publisher.publish(execution_frequency_msg)
        time_step_error_msg = Float64()
        time_step_error_msg.data = self.running_step_time-duration_step
        self.time_step_error_publisher.publish(time_step_error_msg)

        return current_state_idx_lon,current_state_idx_lat, done, info


if __name__ == '__main__':
    #Init nodes and subscribers
    rospy.init_node(node_name)
    landing_simulation_env = LandingSimulationEnv()
    landing_simulation_env.reset()
    action = Vector3()
    action.x = 1
    landing_simulation_env._update_action_values(action),
    landing_simulation_env._step()
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        #do nothing
        rate.sleep()
        
        
    
