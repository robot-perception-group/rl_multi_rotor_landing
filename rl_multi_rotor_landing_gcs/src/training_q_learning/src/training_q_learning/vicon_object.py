'''
Class defining details of the RL task such terminal criteria handling. It is a reduced and adapted version of the landing object class used for simulation.
'''

import rospy
from std_msgs.msg import Float64,Float64MultiArray
from training_q_learning.msg import ObservationRelativeState as ObservationRelativeStateMsg
from training_q_learning.msg import  Action as ActionMsg
from training_q_learning.msg import LandingSimulationObjectState
from training_q_learning.utils import get_publisher
from training_q_learning.parameters import Parameters
import numpy as np
from copy import deepcopy
from training_q_learning.utils import  get_publisher
from training_q_learning.utils_multiresolution import add_cur_step_lims_of_state


#Parameters
node_name = 'landing_simulation_node'

class ViconObject():
    def __init__(self,drone_name):#Add the simulation parameters here
        '''
        Class contains information about the training such as handling of terminal events and reading observations of the environment.
        '''
        # Initialize parameters
        self.parameters = Parameters()
        self.drone_name = drone_name
        self.topic_prefix = '/'+self.drone_name+'/'
        
        #Set up subscribers
        self.observation_continous_subscriber = rospy.Subscriber(self.topic_prefix+'vicon_observation_interface/observations',ObservationRelativeStateMsg,self.read_training_continous_observations)
        self.observation_drone_state_subscriber = rospy.Subscriber(self.topic_prefix+'/vicon/drone/state',LandingSimulationObjectState,self.read_drone_state)

        #Set up publisher
        self.current_idx_publisher = get_publisher(self.topic_prefix+'flight/current_idx',Float64MultiArray,queue_size = 0)
        
        #Initialize observation variables
        self.reset_observation = False
        self.observation_continous = ObservationRelativeStateMsg()
        self.observation_continous_actions = ActionMsg()
        self.observation_drone_state = LandingSimulationObjectState()

        #Initialize the dictionaries containing information about the current discretization
        #Will be updated with actual values during training
        lims_of_cur_steps = dict()  #Dictionary containing the (normalized) positive and negative limit values for each observation specified in the parameters 
        for msg_string in self.parameters.uav_parameters.observation_msg_strings.values():
            add_cur_step_lims_of_state(msg_string,lims_of_cur_steps)   
        self.current_ref_step_counter = 0
        self.lims_of_cur_steps = deepcopy(lims_of_cur_steps)

        #Set up the current state idx
        self.current_state_idx = np.zeros(4,dtype = int) #Integer array containing the idx of the state in the current refinement
        self.previous_state_idx = deepcopy(self.current_state_idx) #Integer array containing the idx of the state in the previous timestep

        #Episode completion variables
        self.done = 0
        self.touchdown_on_platform = False
        self.step_number_in_episode = 0
       
        #Other variables required for execution
        self.action_values = deepcopy(self.parameters.uav_parameters.initial_action_values) #Dictionary in which the current set points for the roll pitch yawrate thrust controller are stored
        self.done_numeric = 0
        self.max_number_of_steps_in_episode = self.parameters.rl_parameters.max_num_timesteps_episode
        self.touchdown_altitude = self.parameters.simulation_parameters.touchdown_altitude
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

        #Number of intervals used for discretization
        n_r = self.parameters.rl_parameters.n_r

        #Determine if discrete goal state has been reached
        goal_state_reached = False

        if n_r % 2 == 0:
            #even number of discretization intervals
            pass
            print("Implementation of even number of intervals not yet tested. Aborting...")
            exit()
        else:
            #uneven number of discretization intervals
            if len(self.parameters.uav_parameters.action_max_values) == 1:
                #If state can be mapped to same curriculum step as in previous timestep
                if self.current_state_idx[0] == self.previous_state_idx[0]: 
                    self.current_cur_step_counter += 1
                    if self.current_cur_step_counter > int(self.parameters.rl_parameters.cur_step_success_duration*(1/self.parameters.rl_parameters.running_step_time)) and self.current_state_idx[0] == len(self.lims_of_cur_steps["rel_p_x"]) - 1 and self.current_state_idx[1] == (n_r-1)/2 and self.current_state_idx[2]  == (n_r-1)/2:
                        goal_state_reached = True
                else:
                    self.current_cur_step_counter = 0
            elif len(self.parameters.uav_parameters.action_max_values) == 2:
                print("Implementation of more than one action not yet testing. Aborting...")
                exit()
            else:
                print("Implementation of more than two actions not yet implemented. Aborting...")
                exit()

        #Determine the reason why an episode needs to be ended
        #Determine the reason why an episode needs to be ended
        done_numeric = 0
        if self.step_number_in_episode >= self.max_number_of_steps_in_episode:
            #Not successful termination
            done_numeric = 1
            self.current_cur_step_counter = 0

        elif self.parameters.simulation_parameters.position_terminal_mode == 'relative' and abs(self.observation_continous.rel_p_x) >= self.parameters.simulation_parameters.max_abs_p_x:
            done_numeric = 3
            self.current_cur_step_counter = 0

        elif self.parameters.simulation_parameters.position_terminal_mode == 'absolute' and abs(self.observation_drone_state.pose.pose.position.x) >= self.parameters.simulation_parameters.max_abs_p_x:
            done_numeric = 3
            self.current_cur_step_counter = 0

        elif self.parameters.simulation_parameters.position_terminal_mode == 'relative' and abs(self.observation_continous.rel_p_y) >= self.parameters.simulation_parameters.max_abs_p_y:
            done_numeric = 4
            self.current_cur_step_counter = 0

        elif self.parameters.simulation_parameters.position_terminal_mode == 'absolute' and abs(self.observation_drone_state.pose.pose.position.y) >= self.parameters.simulation_parameters.max_abs_p_y:
            done_numeric = 4
            self.current_cur_step_counter = 0

        elif goal_state_reached and self.parameters.simulation_parameters.done_criteria["success"]:
            done_numeric = 8
            self.current_cur_step_counter = 0

        elif self.observation_continous.rel_p_z > -self.minimum_altitude and self.parameters.simulation_parameters.done_criteria["minimum_altitude"]:
            done_numeric = 7
            self.current_cur_step_counter = 0

        elif self.mp_contact_occured and self.parameters.simulation_parameters.done_criteria["touchdown_contact"]:
            done_numeric = 9
            self.current_cur_step_counter = 0

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

    def process_data(self):
        """Function checks episode termination status and generates the appropriate reward."""

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
            print('END OF EPISODE: Lateral distance between drone and moving platform too big...')
            # reward = reward_leave_fly_zone 
            done = self.parameters.simulation_parameters.done_criteria["max_lat_distance"]       
        elif done_numeric == 7: 
            print('END OF EPISODE: Minimum altitude reached...')
            # reward = 0
            done = self.parameters.simulation_parameters.done_criteria["minimum_altitude"]
        elif done_numeric == 8:
            done = self.parameters.simulation_parameters.done_criteria["success"]
            # reward = reward_success
            print('SUCCESS: GOAL STATE REACHED')         
        elif done_numeric == 9:
            done = self.parameters.simulation_parameters.done_criteria["touchdown_contact"]
            # reward = 0
            print('END OF EPISODE: Touchdown on platform...')   
        return done, reward
        
if __name__ == '__main__':
    #Init nodes and subscribers
    rospy.init_node(node_name)
    landing_simulation_object = ViconObject()
    
