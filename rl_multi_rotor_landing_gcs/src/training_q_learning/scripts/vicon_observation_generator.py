'''
This script defines a ROS node that computes the relative information between the drone and the 
moving platform for the following values to publish them on the topic 'drone_name/training_observation_interface/observations'
as a ROS message of type ObservationRelativeState:
- Relative position in x-direction (stability axes)
- Relative position in y-direction (stability axes)
- Relative position in z-direction (stability axes)
- Relative velocity in x-direction (stability axes)
- Relative velocity in y-direction (stability axes)
- Relative velocity in z-direction (stability axes)
- Relative acceleration in x-direction (stability axes)
- Relative acceleration in y-direction (stability axes)
- Relative acceleration in z-direction (stability axes)
- Relative yaw angle (stability axes)

For this purpose it reads the relative position and relative velocity from the following topics:
- Relative position: drone_name/landing_simulation/relative_moving_platform_drone/state/pose
- Relative velocity: drone_name/landing_simulation/relative_moving_platform_drone/state/twist
The relative acceleration is determined by applying a first order Butterworth filter to the velocity

Whenever a message is received on one of these topics, the corresponding values are updated in an instance of the class
ObservationRelativeState. 

In the launch file of this node, a frequency can be specified with which this node publishes the aforementioned
ObservationRelativeState message based on the values that are currently stored in the ObservationRelativeState class.
Whenever the message is generated, a consistency check is performed to make sure that the values for the relative 
position, velocity and acceleration are defined in the same coordinate frame.
The launch file also offers the option to define standard deviation that is used to apply zero mean Gaussian noise to 
the observation values
'''


import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Imu
import numpy as np
from training_q_learning.msg import LandingSimulationObjectState
from training_q_learning.msg import ObservationRelativeState as ObservationRelativeStateMsg
from training_q_learning.msg import Action as ActionMsg
from tf.transformations import euler_from_quaternion
from training_q_learning.parameters import Parameters


#Get all training params
parameters = Parameters()

#Parameters
node_name = 'compute_observation_node'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name','iris')   
publish_hz = float(rospy.get_param(rospy.get_namespace()+node_name+'/publish_hz','10'))
std_rel_p_x = float(rospy.get_param(rospy.get_namespace()+node_name+'/std_rel_p_x','0.25'))
std_rel_p_y = float(rospy.get_param(rospy.get_namespace()+node_name+'/std_rel_p_y','0.25'))
std_rel_p_z = float(rospy.get_param(rospy.get_namespace()+node_name+'/std_rel_p_z','0.1'))
std_rel_v_x = float(rospy.get_param(rospy.get_namespace()+node_name+'/std_rel_v_x','0.1'))
std_rel_v_y = float(rospy.get_param(rospy.get_namespace()+node_name+'/std_rel_v_y','0.1'))
std_rel_v_z = float(rospy.get_param(rospy.get_namespace()+node_name+'/std_rel_v_z','0.05'))

#Topic definition subscribers
relative_position_topic = ('landing_simulation/relative_moving_platform_drone/state/pose',PoseStamped)
relative_velocity_topic = ('landing_simulation/relative_moving_platform_drone/state/twist',TwistStamped)
relative_acceleration_topic = ('landing_simulation/relative_moving_platform_drone/state/acceleration',Imu)

#Action setpoint topic
action_to_interface_topic = ('command/action',ActionMsg)

#Topic definition publishers
observation_topic = ('vicon_observation_interface/observations',ObservationRelativeStateMsg)
observation_action_topic = ('vicon_observation_interface/observations_actions',ActionMsg)

#Definition of publishers
relative_observation_publisher = rospy.Publisher(observation_topic[0],observation_topic[1],queue_size = 0)
observation_action_publisher = rospy.Publisher(observation_action_topic[0],observation_action_topic[1],queue_size = 0)

class ObservationRelativeState():
    def __init__(self):
        """Class stores the relative position, relative velocity, relative acceleration and the action setpoints."""
        self.relative_position = PoseStamped()
        self.relative_velocity = TwistStamped()
        self.relative_acceleration = Imu()
        self.action_setpoints = ActionMsg()
        return

    def check_frames(self):
        """Function checks if all sensor values are provided in the same reference frame."""
        frames_equal = False
        ###Use the following line if IMUs are used
        #if self.relative_position.header.frame_id == self.relative_velocity.header.frame_id and self.relative_position.header.frame_id == self.relative_acceleration.header.frame_id and self.relative_velocity.header.frame_id == self.relative_acceleration.header.frame_id:
        frame_ids = [self.relative_position.header.frame_id,self.relative_velocity.header.frame_id,self.relative_acceleration.header.frame_id,self.drone_state.pose.header.frame_id,self.drone_state.twist.header.frame_id]
        if all([x == frame_ids[0] for x in frame_ids]):
            frames_equal = True
        else:    
            rospy.logwarn('Frames are not consistent in observation message:')            
            rospy.logwarn('relative_position frame:')
            rospy.logwarn(self.relative_position.header.frame_id)
            rospy.logwarn('relative_velocity frame:')
            rospy.logwarn(self.relative_velocity.header.frame_id)
            rospy.logwarn('relative_acceleration frame:')
            rospy.logwarn(self.relative_acceleration.header.frame_id)
        return frames_equal    

#Class initialization
observation_relative_state = ObservationRelativeState()

#Define functions
def read_relative_position(msg):
    '''
    Function reads the relative position message and stores the message in the instance of class
    ObservationRelativeState
    '''
    observation_relative_state.relative_position.header.frame_id = msg.header.frame_id
    observation_relative_state.relative_position.header.stamp = msg.header.stamp
    observation_relative_state.relative_position.pose.position.x = msg.pose.position.x + np.random.normal(loc = 0, scale = std_rel_p_x)
    observation_relative_state.relative_position.pose.position.y = msg.pose.position.y + np.random.normal(loc = 0, scale = std_rel_p_y)
    observation_relative_state.relative_position.pose.position.z = msg.pose.position.z + np.random.normal(loc = 0, scale = std_rel_p_z)
    observation_relative_state.relative_position.pose.orientation = msg.pose.orientation
    return

def read_relative_velocity(msg):
    '''
    Function reads the relative velocity message and stores the message in the instance of class
    ObservationRelativeState.
    Function also determines the relative acceleration by numerically differentiating the velocity and applying a first-order Butterworth filter.
    '''
    #Determine header of relative acceleration message
    observation_relative_state.relative_acceleration.header.frame_id = msg.header.frame_id
    observation_relative_state.relative_acceleration.header.stamp = msg.header.stamp

    #Get the relative twist values
    rel_v_x = msg.twist.linear.x + np.random.normal(loc = 0,scale = std_rel_v_x)  
    rel_v_y = msg.twist.linear.y + np.random.normal(loc = 0,scale = std_rel_v_y) 
    rel_v_z = msg.twist.linear.z + np.random.normal(loc = 0,scale = std_rel_v_z)

    #Use discrete first order butterworth filter to reduce noise
    cut_off_ang_freq = 2*np.pi*parameters.uav_parameters.accel_cut_off_freq

    #Determine time between two velocity messages
    delta_t_sample = msg.header.stamp.to_sec()-observation_relative_state.relative_velocity.header.stamp.to_sec()

    #Compute the relative acceleration based on current and previous twist message using first order Butterworth low pass filter
    if delta_t_sample:
        a_x = np.exp(-cut_off_ang_freq*delta_t_sample)*observation_relative_state.relative_acceleration.linear_acceleration.x + (1-np.exp(-cut_off_ang_freq*delta_t_sample))*(rel_v_x - observation_relative_state.relative_velocity.twist.linear.x) / delta_t_sample
        a_y = np.exp(-cut_off_ang_freq*delta_t_sample)*observation_relative_state.relative_acceleration.linear_acceleration.y + (1-np.exp(-cut_off_ang_freq*delta_t_sample))*(rel_v_y - observation_relative_state.relative_velocity.twist.linear.y) / delta_t_sample
        a_z = np.exp(-cut_off_ang_freq*delta_t_sample)*observation_relative_state.relative_acceleration.linear_acceleration.z + (1-np.exp(-cut_off_ang_freq*delta_t_sample))*(rel_v_z - observation_relative_state.relative_velocity.twist.linear.z) / delta_t_sample
        observation_relative_state.relative_acceleration.linear_acceleration.x = a_x
        observation_relative_state.relative_acceleration.linear_acceleration.y = a_y
        observation_relative_state.relative_acceleration.linear_acceleration.z = a_z
    
    #Store the twist message
    observation_relative_state.relative_velocity.header.frame_id = msg.header.frame_id
    observation_relative_state.relative_velocity.header.stamp = msg.header.stamp
    observation_relative_state.relative_velocity.twist.linear.x = rel_v_x
    observation_relative_state.relative_velocity.twist.linear.y = rel_v_y
    observation_relative_state.relative_velocity.twist.linear.z = rel_v_z
    return

def read_relative_acceleration(msg):
    '''
    Function reads the relative acceleration message and stores the message in the instance of class
    ObservationRelativeState.
    This function is not used by default, instead the relative acceleration is derived from the relative twist message.
    '''
    observation_relative_state.relative_acceleration.header.frame_id = msg.header.frame_id
    observation_relative_state.relative_acceleration.linear_acceleration = msg.linear_acceleration
    return

def read_action_setpoints(msg):
    """Function reads the setpoints for the action values of the drone."""
    observation_relative_state.action_setpoints.roll = msg.roll
    observation_relative_state.action_setpoints.pitch = msg.pitch
    observation_relative_state.action_setpoints.yaw = msg.yaw
    observation_relative_state.action_setpoints.v_z = msg.v_z
    return

def compute_relative_observation_msg():
    '''
    Function computes the relative observation message of type ObservationRelativeState. It also
    performs a consitency check to make sure that all values are defined in the same coordinate frame.
    '''
    msg_publish = ObservationRelativeStateMsg()
    msg_publish.header.stamp = rospy.Time.now()
    msg_publish.rel_p_x = observation_relative_state.relative_position.pose.position.x 
    msg_publish.rel_p_y = observation_relative_state.relative_position.pose.position.y 
    msg_publish.rel_p_z = observation_relative_state.relative_position.pose.position.z 
    msg_publish.rel_v_x = observation_relative_state.relative_velocity.twist.linear.x 
    msg_publish.rel_v_y = observation_relative_state.relative_velocity.twist.linear.y 
    msg_publish.rel_v_z = observation_relative_state.relative_velocity.twist.linear.z    
    q = observation_relative_state.relative_position.pose.orientation
    (rel_roll,rel_pitch,rel_yaw) = euler_from_quaternion((q.x,q.y,q.z,q.w))
    msg_publish.rel_yaw = rel_yaw

    #Action setpoint values for drone state
    msg_publish.roll = observation_relative_state.action_setpoints.roll
    msg_publish.pitch = observation_relative_state.action_setpoints.pitch
    msg_publish.yaw = observation_relative_state.action_setpoints.yaw
    msg_publish.v_z = observation_relative_state.action_setpoints.v_z
    msg_publish.roll_rate = float("nan")
    msg_publish.pitch_rate = float("nan")
    msg_publish.yaw_rate = float("nan")

    #Linear acceleration based the derivative of the relative velocity
    msg_publish.rel_a_x = observation_relative_state.relative_acceleration.linear_acceleration.x
    msg_publish.rel_a_y = observation_relative_state.relative_acceleration.linear_acceleration.y
    msg_publish.rel_a_z = observation_relative_state.relative_acceleration.linear_acceleration.z

    #Check if all frames are consistent
    if not observation_relative_state.check_frames():
        msg_publish.header.frame_id = 'FRAME IS NOT CONSISTENT IN OBSERVATION DATASET'
    else:
        msg_publish.header.frame_id = observation_relative_state.relative_position.header.frame_id
    return msg_publish

def compute_observation_action_msg():
    """Function computes the action msg from the current observation that is sent over the ROS network."""    
    msg_publish = ActionMsg()
    msg_publish.roll = observation_relative_state.action_setpoints.roll 
    msg_publish.pitch = observation_relative_state.action_setpoints.pitch 
    msg_publish.yaw = observation_relative_state.action_setpoints.yaw 
    msg_publish.v_z = observation_relative_state.action_setpoints.v_z
    
    #Check if frames are consistent
    if not observation_relative_state.check_frames():
        msg_publish.header.frame_id = 'FRAME IS NOT CONSISTENT IN OBSERVATION DATASET'
    else:
        msg_publish.header.frame_id = observation_relative_state.relative_position.header.frame_id
    return msg_publish

if __name__ == '__main__':
    #Init node
    rospy.init_node(node_name)

    #Define subscribers
    setpoint_action_subscriber = rospy.Subscriber(action_to_interface_topic[0],action_to_interface_topic[1],read_action_setpoints)
    relative_position_subscriber = rospy.Subscriber(relative_position_topic[0],relative_position_topic[1],read_relative_position)
    relative_velocity_subscriber = rospy.Subscriber(relative_velocity_topic[0],relative_velocity_topic[1],read_relative_velocity)
    ### Uncomment the following line when IMUs are used
    #relative_position_subscriber = rospy.Subscriber(relative_acceleration_topic[0],relative_acceleration_topic[1],read_relative_acceleration)

    #Init rate
    rate = rospy.Rate(publish_hz)

    #commands to be executed as long as node is up
    while not rospy.is_shutdown():       
        #Create ROS messages
        relative_observation_msg = compute_relative_observation_msg()
        observation_action_msg = compute_observation_action_msg()

        #Publish ROS messages
        relative_observation_publisher.publish(relative_observation_msg)
        observation_action_publisher.publish(observation_action_msg)

        #Print  info
        rospy.loginfo('Drone name: ')
        rospy.loginfo(drone_name)
        rospy.loginfo('Publish frequency: ')
        rospy.loginfo(publish_hz)
        rate.sleep()