'''

This script defines a ROS node that computes the relative information between the drone and the 
moving platform for the following values to publish them on the topic 'drone_name/training_observation_interface/observations'
as a ROS message of type ObservationRelativeState.msg:
- Relative position in x-direction (stability axes)
- Relative position in y-direction (stability axes)
- Relative position in z-direction (stability axes)
- Relative velocity in x-direction (stability axes)
- Relative velocity in y-direction (stability axes)
- Relative velocity in z-direction (stability axes)
- Relative yaw angle (stability axes)

For this purpose it reads the relative position, velocity and acceleration from the following topics:
- Relative position: drone_name/landing_simulation/relative_moving_platform_drone/state/pose
- Relative velocity: drone_name/landing_simulation/relative_moving_platform_drone/state/twist
- Relative acceleration: drone_name/landing_simulation/relative_moving_platform_drone/state/Imu

Whenever a message is received on one of these topics, the corresponding values are updated in an instance of the class
ObservationRelativeState. 

In the launch file of this node, a frequency can be specified with which this node publishes the aforementioned
ObservationRelativeState message based on the values that are currently stored in the ObservationRelativeState class.
Whenever the message is generated, a consistency check is performed to make sure that the values for the relative 
state, velocity and acceleration are defined in the same coordinate frame.
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
node_name = 'vicon_compute_observation_node'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name','iris')   
publish_hz = float(rospy.get_param(rospy.get_namespace()+node_name+'/publish_hz','10'))
std_rel_p_x = float(rospy.get_param(rospy.get_namespace()+node_name+'/std_rel_p_x','25'))
std_rel_p_y = float(rospy.get_param(rospy.get_namespace()+node_name+'/std_rel_p_y','25'))
std_rel_p_z = float(rospy.get_param(rospy.get_namespace()+node_name+'/std_rel_p_z','2'))
std_rel_v_x = float(rospy.get_param(rospy.get_namespace()+node_name+'/std_rel_p_z','5'))
std_rel_v_y = float(rospy.get_param(rospy.get_namespace()+node_name+'/std_rel_p_z','5'))
std_rel_v_z = float(rospy.get_param(rospy.get_namespace()+node_name+'/std_rel_p_z','2'))


#Topic definition subscribers
drone_state_topic = ('/vicon/drone/state',LandingSimulationObjectState)
moving_platform_state_topic = ('/vicon/moving_platform/state',LandingSimulationObjectState)

relative_position_topic = ('/vicon/relative_moving_platform_drone/state/pose',PoseStamped)
relative_velocity_topic = ('/vicon/relative_moving_platform_drone/state/twist',TwistStamped)
relative_acceleration_topic = ('/vicon/relative_moving_platform_drone/state/acceleration',Imu)

#Action setpoint topic
action_to_interface_topic = ('command/action',ActionMsg)


#Topic definition publishers
observation_topic = ('vicon_observation_interface/observations',ObservationRelativeStateMsg)
observation_action_topic = ('vicon_observation_interface/observations_actions',ActionMsg)



relative_observation_publisher = rospy.Publisher(observation_topic[0],observation_topic[1],queue_size = 0)
observation_action_publisher = rospy.Publisher(observation_action_topic[0],observation_action_topic[1],queue_size = 0)


class VehicleState():
    #Define class and initialize it with some values. These values will be replaced by received data values
    def __init__(self):
        """Class stores the values of a vehicle (pose, twist, attitude)"""
        self.pose = PoseStamped()
        self.pose.header.frame_id = ''
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0
        self.pose.pose.orientation.x = 0
        self.pose.pose.orientation.y = 0
        self.pose.pose.orientation.z = 0  
        self.pose.pose.orientation.w = 1  

        self.roll = 0
        self.pitch = 0
        self.yaw = 0     
        
        self.twist = TwistStamped()
        self.twist.header.frame_id = ''
        self.twist.twist.linear.x = 0
        self.twist.twist.linear.y = 0
        self.twist.twist.linear.z = 0
        self.twist.twist.angular.x = 0
        self.twist.twist.angular.y = 0
        self.twist.twist.angular.z = 0

        return

    def quat2rpy(self):
        euler = euler_from_quaternion([self.pose.pose.orientation.x,self.pose.pose.orientation.y,self.pose.pose.orientation.z,self.pose.pose.orientation.w])
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]


class ObservationRelativeState():
    def __init__(self):
        """Class stores the relative position, relative velocity, relative acceleration, the action setpoints and the drone state."""
        self.relative_position = PoseStamped()
        self.relative_velocity = TwistStamped()
        self.relative_acceleration = Imu()
        self.action_setpoints = ActionMsg()
        self.drone_state = VehicleState()
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
            rospy.logwarn('drone_state pose frame:')
            rospy.logwarn(self.drone_state.pose.header.frame_id)
            rospy.logwarn('drone_state twist frame:')
            rospy.logwarn(self.drone_state.twist.header.frame_id)
            rospy.logwarn('relative_acceleration frame:')
            rospy.logwarn(self.relative_acceleration.header.frame_id)
        return frames_equal    

observation_relative_state = ObservationRelativeState()

#Define functions
def read_drone_state(msg):
    """Function reads the current drone state that is sent over the ROS network."""
    observation_relative_state.drone_state.pose.header.frame_id = msg.header.frame_id
    observation_relative_state.drone_state.pose.pose.position.x = msg.pose.pose.position.x
    observation_relative_state.drone_state.pose.pose.position.y = msg.pose.pose.position.y
    observation_relative_state.drone_state.pose.pose.position.z = msg.pose.pose.position.z
    observation_relative_state.drone_state.pose.pose.orientation.x = msg.pose.pose.orientation.x
    observation_relative_state.drone_state.pose.pose.orientation.y = msg.pose.pose.orientation.y
    observation_relative_state.drone_state.pose.pose.orientation.z = msg.pose.pose.orientation.z
    observation_relative_state.drone_state.pose.pose.orientation.w = msg.pose.pose.orientation.w
    observation_relative_state.drone_state.quat2rpy()

    observation_relative_state.drone_state.twist.header.frame_id = msg.header.frame_id
    observation_relative_state.drone_state.twist.twist.linear.x = msg.twist.twist.linear.vector.x
    observation_relative_state.drone_state.twist.twist.linear.y = msg.twist.twist.linear.vector.y
    observation_relative_state.drone_state.twist.twist.linear.z = msg.twist.twist.linear.vector.z
    observation_relative_state.drone_state.twist.twist.angular.x = msg.twist.twist.angular.vector.x
    observation_relative_state.drone_state.twist.twist.angular.y = msg.twist.twist.angular.vector.y
    observation_relative_state.drone_state.twist.twist.angular.z = msg.twist.twist.angular.vector.z
    return



def read_relative_position(msg):
    '''
    Function reads the relative position message and stores the message in the instance of class
    ObservationRelativeState
    '''
    observation_relative_state.relative_position.header.frame_id = msg.header.frame_id
    observation_relative_state.relative_position.header.stamp = msg.header.stamp
    observation_relative_state.relative_position.pose = msg.pose

    return

def read_relative_velocity(msg):
    '''
    Function reads the relative velocity message and stores the message in the instance of class
    ObservationRelativeState.
    Function also determines the relative acceleration by numerically differentiating the velocity and applying a first order Butterworth filter.
    '''

    observation_relative_state.relative_acceleration.header.frame_id = msg.header.frame_id

    #Use discrete first order butterworth filter to reduce noise
    cut_off_ang_freq = 2*np.pi*parameters.uav_parameters.accel_cut_off_freq
    delta_t_sample = msg.header.stamp.to_sec()-observation_relative_state.relative_velocity.header.stamp.to_sec()
    if delta_t_sample:
        a_x = np.exp(-cut_off_ang_freq*delta_t_sample)*observation_relative_state.relative_acceleration.linear_acceleration.x + (1-np.exp(-cut_off_ang_freq*delta_t_sample))*(msg.twist.linear.x - observation_relative_state.relative_velocity.twist.linear.x) / delta_t_sample
        a_y = np.exp(-cut_off_ang_freq*delta_t_sample)*observation_relative_state.relative_acceleration.linear_acceleration.y + (1-np.exp(-cut_off_ang_freq*delta_t_sample))*(msg.twist.linear.y - observation_relative_state.relative_velocity.twist.linear.y) / delta_t_sample
        a_z = np.exp(-cut_off_ang_freq*delta_t_sample)*observation_relative_state.relative_acceleration.linear_acceleration.z + (1-np.exp(-cut_off_ang_freq*delta_t_sample))*(msg.twist.linear.z - observation_relative_state.relative_velocity.twist.linear.z) / delta_t_sample
        
    #Compute the relative acceleration based on current and previous twist message
        observation_relative_state.relative_acceleration.linear_acceleration.x = a_x
        observation_relative_state.relative_acceleration.linear_acceleration.y = a_y
        observation_relative_state.relative_acceleration.linear_acceleration.z = a_z
    
    #Store the twist message
    observation_relative_state.relative_velocity.header.frame_id = msg.header.frame_id
    observation_relative_state.relative_velocity.header.stamp = msg.header.stamp
    observation_relative_state.relative_velocity.twist = msg.twist
    return

def read_relative_acceleration(msg):
    '''
    Function reads the relative acceleration message and stores the message in the instance of class
    ObservationRelativeState
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
    msg_publish.rel_p_x = observation_relative_state.relative_position.pose.position.x + np.random.normal(loc = 0,scale = std_rel_p_x)
    msg_publish.rel_p_y = observation_relative_state.relative_position.pose.position.y + np.random.normal(loc = 0,scale = std_rel_p_y)
    msg_publish.rel_p_z = observation_relative_state.relative_position.pose.position.z + np.random.normal(loc = 0,scale = std_rel_p_z)
    msg_publish.rel_v_x = observation_relative_state.relative_velocity.twist.linear.x + np.random.normal(loc = 0,scale = std_rel_v_x)
    msg_publish.rel_v_y = observation_relative_state.relative_velocity.twist.linear.y + np.random.normal(loc = 0,scale = std_rel_v_y)
    msg_publish.rel_v_z = observation_relative_state.relative_velocity.twist.linear.z  + np.random.normal(loc = 0,scale =std_rel_v_z)   
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
    
    if not observation_relative_state.check_frames():
        msg_publish.header.frame_id = 'FRAME IS NOT CONSISTENT IN OBSERVATION DATASET'
    else:
        msg_publish.header.frame_id = observation_relative_state.relative_position.header.frame_id
    return msg_publish



if __name__ == '__main__':
    #Init nodes and subscribers
    rospy.init_node(node_name)
    drone_state_subscriber = rospy.Subscriber(drone_state_topic[0],drone_state_topic[1],read_drone_state)
    setpoint_action_subscriber = rospy.Subscriber(action_to_interface_topic[0],action_to_interface_topic[1],read_action_setpoints)

    #Define subscriber
    relative_position_subscriber = rospy.Subscriber(relative_position_topic[0],relative_position_topic[1],read_relative_position)
    relative_velocity_subscriber = rospy.Subscriber(relative_velocity_topic[0],relative_velocity_topic[1],read_relative_velocity)
    ### Uncomment the following line when IMUs are used
    #relative_position_subscriber = rospy.Subscriber(relative_acceleration_topic[0],relative_acceleration_topic[1],read_relative_acceleration)

    #Init rate
    rate = rospy.Rate(publish_hz)

    #commands to be executed as long as node is up
    while not rospy.is_shutdown():       
        #Create ROS message
        relative_observation_msg = compute_relative_observation_msg()
        #Publish ROS message
        relative_observation_publisher.publish(relative_observation_msg)
        observation_action_msg = compute_observation_action_msg()
        observation_action_publisher.publish(observation_action_msg)
        #Print out info
        rospy.loginfo('Drone name: ')
        rospy.loginfo(drone_name)
        rospy.loginfo('Publish frequency: ')
        rospy.loginfo(publish_hz)
        rate.sleep()