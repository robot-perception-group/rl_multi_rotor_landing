'''
This script sets up a ROS node that tracks information about the landing procedure and stores them in a csv file so that the 
landing procedure can be analyzed in a later step.
'''

import rospy
from std_msgs.msg import Float64MultiArray, Bool, Int64, Float64
import numpy as np
from training_q_learning.parameters import Parameters
from training_q_learning.utils import get_publisher
from training_q_learning.msg import ObservationRelativeState
from training_q_learning.msg import LandingSimulationObjectState
from gazebo_msgs.msg import ModelState
import csv 


#Define script parameters
parameters = Parameters()
node_name = 'analysis_node_2D'
max_num_timesteps_episode = parameters.rl_parameters.max_num_timesteps_episode
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name','iris') 
topic_prefix = '/'+drone_name+'/'

#Topics
#Read data topics
reset_simulation_topic = (topic_prefix + 'training/init_reset_simulation',Bool)
observation_topic = (topic_prefix+'training_observation_interface/observations',ObservationRelativeState)
current_state_idx_x_topic = (topic_prefix+'training/current_idx_x',Float64MultiArray)
current_state_idx_y_topic = (topic_prefix+'training/current_idx_y',Float64MultiArray)
moving_platform_state_topic = (topic_prefix + 'landing_simulation/world_frame/moving_platform/state',LandingSimulationObjectState)
drone_state_topic = (topic_prefix + 'landing_simulation/world_frame/drone/state',LandingSimulationObjectState)

#Publish topics
current_timestep_x_topic = (topic_prefix + 'analysis_node/current_timestep_x',Int64)
max_ref_step_percentage_x_topic = (topic_prefix + 'analysis_node/max_ref_step_percentage_x',Float64)
current_timestep_y_topic = (topic_prefix + 'analysis_node/current_timestep_y',Int64)
max_ref_step_percentage_y_topic = (topic_prefix + 'analysis_node/max_ref_step_percentage_y',Float64)

#Publishers
current_timestep_x_publisher = get_publisher(current_timestep_x_topic[0],current_timestep_x_topic[1],queue_size = 0)
max_ref_step_percentage_x_publisher = get_publisher(max_ref_step_percentage_x_topic[0],max_ref_step_percentage_x_topic[1],queue_size = 0)
current_timestep_y_publisher = get_publisher(current_timestep_y_topic[0],current_timestep_y_topic[1],queue_size = 0)
max_ref_step_percentage_y_publisher = get_publisher(max_ref_step_percentage_y_topic[0],max_ref_step_percentage_y_topic[1],queue_size = 0)



# Script variables 
platform_edge_length_x = 1
platform_edge_length_y = 1

#csv file names
file_path_csv = '/home/pgoldschmid/Desktop/exp/vmp_0_4/test_results/test_data_vmp_0_4_sim_1_vmpexp_x_0_vmpexp_y_0_rmp.csv'

print("Begin / proceed logging...")

#Class definition
class AnalysisNode():
    def __init__(self):
        """Class sets up the required functions to log data about landing performance when two instances of the same agent are used to control in motion in the longitudinal and lateral direction."""

        #Define variables 
        dim_observation_space = len(parameters.uav_parameters.observation_msg_strings)       
        self.current_state_idx_x = np.zeros(dim_observation_space+dim_observation_space+1)
        self.current_state_idx_y = np.zeros(dim_observation_space+dim_observation_space+1)
        self.current_timestep_x = 0
        self.current_timestep_y = 0
        self.current_ref_step_idx_x = 0
        self.current_ref_step_idx_y = 0
        self.max_ref_step_idx_x = len(parameters.rl_parameters.discretization_steps["rel_p_x"])-2
        self.max_ref_step_idx_y = len(parameters.rl_parameters.discretization_steps["rel_p_y"])-2
        self.max_ref_step_counter_x = 0
        self.max_ref_step_counter_y = 0
        self.max_ref_step_percentage_x = 0
        self.max_ref_step_percentage_y = 0
        self.sum_rel_p_x = 0
        self.sum_rel_p_y = 0
        self.sum_rel_p_z = 0
        self.sum_rel_v_x = 0
        self.sum_rel_v_y = 0
        self.sum_rel_v_z = 0
        self.success = 0
        self.ep_number = 0
        self.observation = ObservationRelativeState()
        self.absolute_moving_platform_state = ModelState()
        self.absolute_drone_state = ModelState()

        #Set flag if current_state_idx has never changed during test.
        self.change_flag_x = 0
        self.change_flag_y = 0

        #Logged parameters
        self.mean_rel_p_x = 0
        self.mean_rel_p_y = 0
        self.mean_rel_p_z = 0
        self.mean_rel_v_x = 0
        self.mean_rel_v_y = 0
        self.mean_rel_v_z = 0

        #Information about moving platform and drone in world frame
        self.mp_p_x = 0
        self.mp_p_y = 0
        self.mp_p_z = 0
        self.mp_v_x = 0
        self.mp_v_y = 0
        self.mp_v_z = 0
        self.drone_p_x = 0
        self.drone_p_y = 0
        self.drone_p_z = 0
        self.drone_v_x = 0
        self.drone_v_y = 0
        self.drone_v_z = 0
        return

    def publish_stats_x(self):
        """Function publishes basic statistics over the ROS framework for the movement in longitudinal direction. """
        msg_current_timestep_x = Int64()
        msg_current_timestep_x.data = int(self.current_timestep_x)
        current_timestep_x_publisher.publish(msg_current_timestep_x)

        msg_max_ref_step_percentage_x = Float64()
        msg_max_ref_step_percentage_x.data = self.max_ref_step_percentage_x
        max_ref_step_percentage_x_publisher.publish(msg_max_ref_step_percentage_x)
        return

    def publish_stats_y(self):
        """Function publishes basic statistics over the ROS framework for the movement in lateral direction. """
        msg_current_timestep_y = Int64()
        msg_current_timestep_y.data = int(self.current_timestep_y)
        current_timestep_y_publisher.publish(msg_current_timestep_y)

        msg_max_ref_step_percentage_y = Float64()
        msg_max_ref_step_percentage_y.data = self.max_ref_step_percentage_y
        max_ref_step_percentage_y_publisher.publish(msg_max_ref_step_percentage_y)
        return

    def log_stats_x(self):
        """Function logs and computes basic statistics for motion in x-direction."""
        #Count how many timesteps where spend in the latest refinement step
        if self.current_ref_step_idx_x == self.max_ref_step_idx_x:
            self.max_ref_step_counter_x += 1

        #Increment current timestep in episode
        self.current_timestep_x += 1

        #Compute the percentage spend in the max. ref. step
        self.max_ref_step_percentage_x = self.max_ref_step_counter_x / self.current_timestep_x

        #Compute the mean values
        self.sum_rel_p_x += self.observation.rel_p_x 
        self.sum_rel_v_x += self.observation.rel_v_x 
        self.mean_rel_p_x = self.sum_rel_p_x / self.current_timestep_x
        self.mean_rel_v_x = self.sum_rel_v_x / self.current_timestep_x
        return

    def log_stats_y(self):
        """Function logs and computes basic statistics for motion in y-direction."""
        #Count how many timesteps where spend in the latest refinement step
        if self.current_ref_step_idx_y == self.max_ref_step_idx_y:
            self.max_ref_step_counter_y += 1

        #Increment current timestep in episode
        self.current_timestep_y += 1

        #Compute the percentage spend in the max. ref. step
        self.max_ref_step_percentage_y = self.max_ref_step_counter_y / self.current_timestep_y

        #Compute the mean values
        self.sum_rel_p_y += self.observation.rel_p_y 
        self.sum_rel_v_y += self.observation.rel_v_y 
        self.mean_rel_p_y = self.sum_rel_p_y / self.current_timestep_y
        self.mean_rel_v_y = self.sum_rel_v_y / self.current_timestep_y
        return

    def print_stats(self):
        '''
        Function prints statistics to the terminal window
        '''
        print("Episode length: ",np.max([self.current_timestep_x,self.current_timestep_y]))
        print("Percentage in max. ref. step x:",self.max_ref_step_percentage_x*100)
        print("Percentage in max. ref. step y:",self.max_ref_step_percentage_y*100)
        print("Mean rel_p_x: ",self.mean_rel_p_x)
        print("Mean rel_p_y: ",self.mean_rel_p_y)
        print("Mean rel_v_x: ",self.mean_rel_v_x)
        print("Mean rel_v_y: ",self.mean_rel_v_y)
        print("========================================")
        return

    def read_current_state_idx_x(self,msg):
        """Function reads the current discrete states of the longitudinal motion."""
        #If the state has changed
        if not msg.data == self.current_state_idx_x:
            self.change_flag_x += 1
        self.current_ref_step_idx_x = msg.data[0]   
        self.current_state_idx_x = msg.data
        self.log_stats_x()
        self.publish_stats_x()
        return

    def read_current_state_idx_y(self,msg):
        """Function reads the current discrete states of the lateral motion."""
        #If the state has changed
        if not msg.data == self.current_state_idx_y:
            self.change_flag_y += 1
        self.current_ref_step_idx_y = msg.data[0]   
        self.current_state_idx_y = msg.data     
        self.log_stats_y()
        self.publish_stats_y()
        return

    def read_reset_simulation(self,msg):
        """Function reads the reset simulation topic and prints, writes and resets statistics."""
        self.print_stats()
        self.write_status_to_csv()
        self.reset_stats()
        return

    def read_absolute_moving_platform_state(self,msg):
        """Function reads the current state of the moving platform in the earth frame."""
        self.mp_p_x = msg.pose.pose.position.x
        self.mp_p_y = msg.pose.pose.position.y
        self.mp_p_z = msg.pose.pose.position.z
        self.mp_v_x = msg.twist.twist.linear.vector.x
        self.mp_v_y = msg.twist.twist.linear.vector.y
        self.mp_v_z = msg.twist.twist.linear.vector.z
        return
    
    def read_absolute_drone_state(self,msg):
        """Function reads the current state of the drone in the earth frame."""
        self.drone_p_x = msg.pose.pose.position.x
        self.drone_p_y = msg.pose.pose.position.y
        self.drone_p_z = msg.pose.pose.position.z
        self.drone_v_x = msg.twist.twist.linear.vector.x
        self.drone_v_y = msg.twist.twist.linear.vector.y
        self.drone_v_z = msg.twist.twist.linear.vector.z
        return

    def read_observation(self,msg):
        """Function reads the observation of the environment and determines landing success."""
        self.observation = msg
        if abs(self.observation.rel_p_x) < platform_edge_length_x/2 and abs(self.observation.rel_p_y) < platform_edge_length_y/2:
            self.success = 1
        else:
            self.success = 0
        return

    def reset_stats(self):
        """Function resets statistics."""
        #perform logging to include the reset timestep in logged data
        self.log_stats_x()
        self.log_stats_y()

        #reset values
        self.success = 0
        
        self.current_timestep_x = 0
        self.current_ref_step_idx_x = 0
        self.max_ref_step_counter_x = 0           
        self.max_ref_step_percentage_x = 0

        self.current_timestep_y = 0
        self.current_ref_step_idx_y = 0
        self.max_ref_step_counter_y = 0           
        self.max_ref_step_percentage_y = 0

        self.change_flag_x = 0
        self.change_flag_y = 0

        self.sum_rel_p_x = 0
        self.sum_rel_p_y = 0
        self.sum_rel_p_z = 0
        self.sum_rel_v_x = 0
        self.sum_rel_v_y = 0
        self.sum_rel_v_z = 0

        self.mean_rel_p_x = 0
        self.mean_rel_p_y = 0
        self.mean_rel_p_z = 0
        self.mean_rel_v_x = 0
        self.mean_rel_v_y = 0
        self.mean_rel_v_z = 0
        return

    def write_status_to_csv(self):
        """Function writes current status of the landing scenario to a csv file."""
        print("current_state_idx_x =",self.current_state_idx_x)
        print("current_state_idx_y =",self.current_state_idx_y)
        self.ep_number += 1
        row = []
        row += [self.ep_number]
        row += [self.current_state_idx_x[0],self.current_state_idx_x[1],self.current_state_idx_x[2]]
        row += [self.current_timestep_x]
        row += [self.current_state_idx_y[0],self.current_state_idx_y[1],self.current_state_idx_y[2]]
        row += [self.current_timestep_y]
        row += [getattr(self.observation,"rel_p_x"),getattr(self.observation,"rel_p_y"),getattr(self.observation,"rel_p_z")]
        row += [getattr(self.observation,"rel_v_x"),getattr(self.observation,"rel_v_y"),getattr(self.observation,"rel_v_z")]
        row += [self.mp_p_x,self.mp_p_y,self.mp_p_z]
        row += [self.mp_v_x,self.mp_v_y,self.mp_v_z]
        row += [self.drone_p_x,self.drone_p_y,self.drone_p_z]
        row += [self.drone_v_x,self.drone_v_y,self.drone_v_z]
        row += [self.change_flag_x,self.change_flag_y]
        row += [self.success]

        with open(file_path_csv,'a') as f:
            writer = csv.writer(f)
            writer.writerow(row)
            print("added csv entry")
        print(row)
        return

analysis_node = AnalysisNode()

if __name__ == '__main__':
    #Init nodes and subscribers
    rospy.init_node(node_name)
    current_state_idx_x_subscriber = rospy.Subscriber(current_state_idx_x_topic[0],current_state_idx_x_topic[1],analysis_node.read_current_state_idx_x)
    reset_simulation_subscriber = rospy.Subscriber(reset_simulation_topic[0],reset_simulation_topic[1],analysis_node.read_reset_simulation)
    current_state_idx_y_subscriber = rospy.Subscriber(current_state_idx_y_topic[0],current_state_idx_y_topic[1],analysis_node.read_current_state_idx_y)
    observation_subscriber = rospy.Subscriber(observation_topic[0],observation_topic[1],analysis_node.read_observation)
    moving_platform_state_subscriber = rospy.Subscriber(moving_platform_state_topic[0],moving_platform_state_topic[1],analysis_node.read_absolute_moving_platform_state)
    drone_state_subscriber = rospy.Subscriber(drone_state_topic[0],drone_state_topic[1],analysis_node.read_absolute_drone_state)
    rospy.spin()
    

   



