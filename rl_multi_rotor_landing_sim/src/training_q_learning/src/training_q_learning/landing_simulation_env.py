"""
Script contains the definition of the class defining the training environment and some of its interfaces to other ros nodes.
Furthermore, it registers the landing scenario as an environment in gym.
"""

import gym
import rospy
import numpy as np
import time
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
from training_q_learning.utils_multiresolution import get_discrete_state_from_ros_msg
from std_msgs.msg import Bool
from copy import deepcopy
from training_q_learning.srv import ResetRandomSeed

#Script variables
node_name = 'landing_simulation_gym_node'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name','iris')

#Topic definitions
action_to_interface_topic = ('training_action_interface/action_to_interface',Action)
reset_simulation_topic = ("training/reset_simulation",Bool)
init_reset_simulation_topic = ("training/init_reset_simulation",Bool)
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
        self.drone_name = drone_name
        
        
        #Set up publishers 
        self.action_to_interface_publisher = get_publisher(action_to_interface_topic[0],action_to_interface_topic[1],queue_size = 0)
        self.init_reset_simulation_publisher = get_publisher(init_reset_simulation_topic[0],init_reset_simulation_topic[1],queue_size = 0)
        self.reset_simulation_publisher = get_publisher(reset_simulation_topic[0],reset_simulation_topic[1],queue_size = 0)
        self.step_execution_frequency_publisher = get_publisher(step_execution_frequency_topic[0],step_execution_frequency_topic[1],queue_size = 0)
        self.time_step_error_publisher = get_publisher(timestep_error_topic[0],timestep_error_topic[1],queue_size = 0)
        self.reward_publisher = get_publisher(reward_topic[0],reward_topic[1],queue_size = 0)

        #Set up services
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
        rospy.wait_for_service('/moving_platform/reset_random_seed')
        self.mp_reset_randon_seed = rospy.ServiceProxy('/moving_platform/reset_random_seed',ResetRandomSeed)
    
        #Set the random number seed used for initializing the drone
        print("Set seed for initial values to ",self.parameters.rl_parameters.seed_init)
        np.random.seed(self.parameters.rl_parameters.seed_init)
        print("Send signal to reset random number generator to moving platform trajectory generator ")
        self.mp_reset_randon_seed(str(self.parameters.rl_parameters.seed_init))

        #Initialize object defining the learning behavior
        self.landing_simulation_object = LandingSimulationObject(self.drone_name,self.parameters)

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
        self.landing_simulation_object.touchdown_on_platform = False
        self.exploration_rate = 0
        
        #Set step parameters
        self.running_step_time = self.parameters.rl_parameters.running_step_time

        #Set parameters required for initilaization of drone after reset
        self.max_x = self.parameters.simulation_parameters.init_max_x
        self.min_x = self.parameters.simulation_parameters.init_min_x
        self.max_y = self.parameters.simulation_parameters.init_max_y
        self.min_y = self.parameters.simulation_parameters.init_min_y
        self.max_z = self.parameters.simulation_parameters.init_max_z
        self.min_z = self.parameters.simulation_parameters.init_min_z
        self.init_altitude = self.parameters.simulation_parameters.init_altitude

        #Other script variables
        self.test_mode_activated = False
        self.log_dir_path = None
        self.date_prefix = None

        print("Done setting up training environment...")
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

        #Pass the updated values to the landing_simulation_object
        self.landing_simulation_object.action_values = deepcopy(self.action_values)
        return


    def reset(self): 
        '''Function resets the training environment and updates logging data '''

        if self.test_mode_activated:
            self.send_init_reset_simulation_signal(True)
            #Give time to send and process reset signal in the analysis node if test mode is activated
            self.unpause_sim()
            rospy.sleep(0.2)
            self.pause_sim()

        #Reset the setpoints for the low-level controllers of the copter
        self.action_values = deepcopy(self.parameters.uav_parameters.initial_action_values)
        self.publish_action_to_interface()

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
                #Get parameters specifying the normal distribution
                init_mu_x = self.parameters.simulation_parameters.init_mu_x 
                init_sigma_x = self.parameters.simulation_parameters.init_sigma_x
                init_mu_y = self.parameters.simulation_parameters.init_mu_y 
                init_sigma_y = self.parameters.simulation_parameters.init_sigma_y
                
                #Draw init value from normal distribution
                x_init = np.random.normal(init_mu_x,init_sigma_x)
                y_init = np.random.normal(init_mu_y,init_sigma_y)   
        else:
            print("Only 1D case is implemented. Aborting...")
            exit()

        #Compute the init position within the specified fly zone ('absolute') or relative to the moving platform ('relative'). Clip to make sure the drone is not initialized outside the flyzone
        if self.parameters.simulation_parameters.init_mode == 'relative':
            init_drone.pose.position.x = np.clip(x_init + object_coordinates_moving_platform.pose.position.x,-self.parameters.simulation_parameters.max_abs_p_x,self.parameters.simulation_parameters.max_abs_p_y)
            init_drone.pose.position.y = np.clip(y_init + object_coordinates_moving_platform.pose.position.y,-self.parameters.simulation_parameters.max_abs_p_y,self.parameters.simulation_parameters.max_abs_p_y)
        
        elif self.parameters.simulation_parameters.init_mode == 'absolute':
            init_drone.pose.position.x = np.clip(x_init,-self.parameters.simulation_parameters.max_abs_p_x,self.parameters.simulation_parameters.max_abs_p_x) 
            init_drone.pose.position.y = np.clip(y_init,-self.parameters.simulation_parameters.max_abs_p_y,self.parameters.simulation_parameters.max_abs_p_y) 

        #Define the new init state (init velocity is 0)
        init_drone.pose.position.z = self.init_altitude
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
        object_coordinates_drone_after_reset = self.model_coordinates(self.drone_name,"world")
        self.pause_sim()
        
        #Collect the new observation
        self.landing_simulation_object.reset_observation = True
        observation_msg = self.landing_simulation_object.get_observation()
        self.landing_simulation_object.reset_observation = False
        observation = self.convert_observation_msg(observation_msg)

        #Execute reward function once to update the time dependent components in the reward function, when a training and not a test is running
        if self.test_mode_activated == False:
            _ = self.landing_simulation_object.compute_reward()

        #Store the init values if a log dir path has been set 
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

        else:
            #If the path is None / False, then init data is not stored
            pass

        #Update the parameters required to run the simulation
        self.episode += 1
        self.step_number_in_episode = 0
        self.landing_simulation_object.step_number_in_episode = 0
        self.episode_reward = 0
        self.landing_simulation_object.cum_reward = 0
        self.landing_simulation_object.done_numeric = 0
        self.reset_happened = True

        #Send reset signal to ROS network
        self.send_reset_simulation_signal(True)

        return observation
        
    
    def send_init_reset_simulation_signal(self,status: Bool):
        '''Function sends out a boolean value indicating that a reset has been requested. This can be used in other nodes that need reset, such as the analysis node. '''
        msg_reset = Bool()
        msg_reset.data = True
        self.init_reset_simulation_publisher.publish(msg_reset)
        return

    def send_reset_simulation_signal(self,status: Bool):
        '''Function sends out a boolean value indicating that a reset has been comnpleted. This can be used in other nodes that need reset, such as the action to training interface node. '''
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
        self.update_action_values(action)        
        self.publish_action_to_interface()
        
        #Let the simulation run for one RL timestep and measure the wall clock time that has passed during that timestep
        self.unpause_sim()
        t_start_f = time.time()
        rospy.sleep(self.running_step_time)
        t_stop_f = time.time()
        self.pause_sim()

        #Get observation
        observation_msg_list = self.landing_simulation_object.get_observation()
       
        #Normalize and clip the observations
        observation = self.convert_observation_msg(observation_msg_list)

        #Map the current observation to a discrete state value
        self.landing_simulation_object.current_state_idx = get_discrete_state_from_ros_msg(observation[0],observation[1],self.landing_simulation_object.lims_of_cur_steps,self.landing_simulation_object.n_r,self.parameters)

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
        duration_step = t_stop_f-t_start_f
        execution_frequency_msg = Float64()
        execution_frequency_msg.data = 1/duration_step
        self.step_execution_frequency_publisher.publish(execution_frequency_msg)
        time_step_error_msg = Float64()
        time_step_error_msg.data = self.running_step_time-duration_step
        self.time_step_error_publisher.publish(time_step_error_msg)
        return observation, reward, done, info


    def step_2D(self,action_lon,action_lat,parameters_lon,parameters_lat,lims_of_cur_steps_lon,lims_of_cur_steps_lat):
        """Function performs one time step for the scenario in which two instances of the same agent are used to independently control longitudinal and lateral motion of the multi-rotor vehicle assuming a symmetric vehicle."""
        #Reset touchdown_value if previous step was the reset step
        if self.reset_happened == True:
            self.landing_simulation_object.touchdown_on_platform = False
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
        self.unpause_sim()
        t_start_f = time.time()
        rospy.sleep(self.running_step_time)
        t_stop_f = time.time()
        self.pause_sim()

        #Get observation
        observation_msg_list = self.landing_simulation_object.get_observation()

        #Process the results of the timestep for longitudinal motion
        self.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
        self.landing_simulation_object.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
        observation_lon = self.convert_observation_msg(observation_msg_list)
        current_state_idx_lon =  get_discrete_state_from_ros_msg(observation_lon[0],observation_lon[1],lims_of_cur_steps_lon,self.landing_simulation_object.n_r,self.parameters)
        self.landing_simulation_object.current_state_idx = deepcopy(current_state_idx_lon)
        (done_lon,_) = self.landing_simulation_object.process_data() 

        #Process the results of the timestep for lateral motion
        self.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
        self.landing_simulation_object.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
        observation_lat = self.convert_observation_msg(observation_msg_list)
        setattr(observation_lat[0],"rel_p_y",-getattr(observation_lat[0],"rel_p_y"))
        setattr(observation_lat[0],"rel_v_y",-getattr(observation_lat[0],"rel_v_y"))
        setattr(observation_lat[0],"rel_a_y",-getattr(observation_lat[0],"rel_a_y"))
        current_state_idx_lat =  get_discrete_state_from_ros_msg(observation_lat[0],observation_lat[1],lims_of_cur_steps_lat,self.landing_simulation_object.n_r,self.parameters)
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
        self.landing_simulation_object.step_number_in_episode = self.step_number_in_episode
        info = {}

        #Publish results of time measurement
        duration_step = t_stop_f-t_start_f
        execution_frequency_msg = Float64()
        execution_frequency_msg.data = 1/duration_step
        self.step_execution_frequency_publisher.publish(execution_frequency_msg)
        time_step_error_msg = Float64()
        time_step_error_msg.data = self.running_step_time-duration_step
        self.time_step_error_publisher.publish(time_step_error_msg)

        return current_state_idx_lon,current_state_idx_lat, done, info
    

    def external_controller_step(self,freq):
        #Let simulation run for one timestep and determine the elapsed time using the wall clock
        self.unpause_sim()
        t_start_f = time.time()
        rospy.sleep(1/freq)
        t_stop_f = time.time()
        self.pause_sim()

        (done,_) = self.landing_simulation_object.process_data()

        #Publish results of time measurement
        duration_step = t_stop_f-t_start_f
        execution_frequency_msg = Float64()
        execution_frequency_msg.data = 1/duration_step
        self.step_execution_frequency_publisher.publish(execution_frequency_msg)
        time_step_error_msg = Float64()
        time_step_error_msg.data = self.running_step_time-duration_step
        self.time_step_error_publisher.publish(time_step_error_msg)
        return done


if __name__ == '__main__':
    #Init nodes and subscribers
    rospy.init_node(node_name)
    landing_simulation_env = LandingSimulationEnv()
    landing_simulation_env.reset()
    action = Vector3()
    action.x = 1
    landing_simulation_env.update_action_values(action),
    landing_simulation_env._step()
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        #do nothing
        rate.sleep()
        
        
    
