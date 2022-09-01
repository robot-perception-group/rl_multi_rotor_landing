''' 
This script creates an interface node which fuses the setpoints for roll pitch yaw rate and vertical velocity  and which arrive over different 
topics into the RollPitchYawrateThrust message that is required by the roll pitch yawrate thrust controller of the RotorSimulator package.

For this purpose it reads the actions commanded by the RL agent and updates the setpoint values for roll, pitch, v_z and yaw accordingly.
For v_z and the yaw angle, it publishes the corresponding setpoint values on different topics. These values are then read by 
PID controllers for v_z and the yaw angle that produce as output a thrust command or a yaw_rate.
These control efforts are read by this node and the values are also stored.

Using a publish frequency defined in the launch file, this node creates a roll pitch yaw rate thrust message on the basis of all 
values, that are currently stored.
'''

import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import  PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from training_q_learning.msg import Action
from mav_msgs.msg import RollPitchYawrateThrust
from geometry_msgs.msg import Vector3
from training_q_learning.msg import LandingSimulationObjectState
from training_q_learning.parameters import Parameters
from copy import deepcopy
from tf.transformations import euler_from_quaternion


#Define topics
action_to_interface_topic = ('training_action_interface/action_to_interface',Action)
v_x_pid_setpoint_topic = ('training_action_interface/setpoint/v_x',Float64)
v_y_pid_setpoint_topic = ('training_action_interface/setpoint/v_y',Float64)
v_z_pid_setpoint_topic = ('training_action_interface/setpoint/v_z',Float64)
yaw_pid_setpoint_topic = ('training_action_interface/setpoint/yaw',Float64)

v_x_pid_state_topic = ('training_action_interface/state/v_x',Float64)
v_y_pid_state_topic = ('training_action_interface/state/v_y',Float64)
v_z_pid_state_topic = ('training_action_interface/state/v_z',Float64)
yaw_pid_state_topic = ('training_action_interface/state/yaw',Float64)

v_x_pid_control_effort_topic = ('training_action_interface/control_effort/v_x',Float64)
v_y_pid_control_effort_topic = ('training_action_interface/control_effort/v_y',Float64)
v_z_pid_control_effort_topic = ('training_action_interface/control_effort/v_z',Float64)
yaw_pid_control_effort_topic = ('training_action_interface/control_effort/yaw',Float64)

landing_simulation_object_state_topic = ('landing_simulation/world_frame/drone/state/',LandingSimulationObjectState)
pose_topic = ('landing_simulation/relative_moving_platform_drone/state/pose',PoseStamped)
twist_topic = ('landing_simulation/relative_moving_platform_drone/state/twist',TwistStamped)
roll_pitch_yawrate_thrust_topic = ('command/roll_pitch_yawrate_thrust',RollPitchYawrateThrust)
reset_simulation_topic = ('training/reset_simulation',Bool)

#Define parameters for ROS
node_name = 'training_action_interface_node'
publish_hz = float(rospy.get_param(rospy.get_namespace()+node_name+'/publish_hz','10'))

#Define publishers
v_z_pid_setpoint_publisher = rospy.Publisher(v_z_pid_setpoint_topic[0],v_z_pid_setpoint_topic[1],queue_size = 3)
v_z_pid_state_publisher = rospy.Publisher(v_z_pid_state_topic[0],v_z_pid_state_topic[1],queue_size = 3)
yaw_pid_setpoint_publisher = rospy.Publisher(yaw_pid_setpoint_topic[0],yaw_pid_setpoint_topic[1],queue_size = 3)
yaw_pid_state_publisher = rospy.Publisher(yaw_pid_state_topic[0],yaw_pid_state_topic[1],queue_size = 3)
roll_pitch_yawrate_thrust_publisher = rospy.Publisher(roll_pitch_yawrate_thrust_topic[0],roll_pitch_yawrate_thrust_topic[1],queue_size = 3)

#Define classes
class Interface():
    def __init__(self):
        '''
        Class contains the variables necessary to provide setpoints to the roll pitch yaw rate thrust controller.
        Furthermore, it serves as an interface to store the setpoint values (the input) of the PID controller for v_z and yaw as well
        as the control_effort values (the output) of the aforementioned PID controllers.
        '''
        self.v_z_control_effort = 0
        self.v_z_state = 0
        self.action_values = deepcopy(parameters.uav_parameters.initial_action_values)

        self.yaw_control_effort = 0
        self.yaw_state = 0


#Define class instances
parameters = Parameters()
interface = Interface()
action_values_start = deepcopy(parameters.uav_parameters.initial_action_values)

def read_v_z_control_effort(msg):
    '''
    Function reads the control effort message that is output by the PID controller for v_z
    '''
    interface.v_z_control_effort = msg.data
    return 

def read_yaw_control_effort(msg):
    '''
    Function reads the control effort message that is output by the PID controller for yaw
    '''
    interface.yaw_control_effort = msg.data
    return 

def read_pose(msg):
    '''
    Function reads the message containing the relative pose information of moving platform and drone.
    '''
    q = msg.pose.orientation
    (roll,pitch,yaw) = euler_from_quaternion([q.x,q.y,q.z,q.w])
    interface.yaw_state = yaw
    msg_yaw_state = Float64()
    msg_yaw_state.data = interface.yaw_state
    yaw_pid_state_publisher.publish(msg_yaw_state)
    return

def read_twist(msg):
    '''
    Function reads the message containing the relative pose information of moving platform and drone.
    '''
    interface.v_z_state = -msg.twist.linear.z
    msg_v_z_state = Float64()
    msg_v_z_state.data = interface.v_z_state
    v_z_pid_state_publisher.publish(msg_v_z_state)
    return


def read_landing_simulation_object_state(msg):
    '''
    Function reads landing simulation object message to extract the relative v_z value 
    and the relative yaw angle.
    '''
    interface.v_z_state = msg.twist.twist.linear.vector.z
    q = msg.pose.pose.orientation
    (roll,pitch,yaw) = euler_from_quaternion([q.x,q.y,q.z,q.w])

    interface.yaw_state = yaw
    
    msg_v_z_state = Float64()
    msg_v_z_state.data = interface.v_z_state
    v_z_pid_state_publisher.publish(msg_v_z_state)

    msg_yaw_state = Float64()
    msg_yaw_state.data = interface.yaw_state
    yaw_pid_state_publisher.publish(msg_yaw_state)
    return



def publish_setpoint_v_z():
    '''
    Function publishes the setpoint for the PID controller of v_z based on the current setpoint value for v_z
    '''
    msg = Float64()
    msg.data = interface.action_values["v_z"]
    v_z_pid_setpoint_publisher.publish(msg)
    return

def publish_setpoint_yaw():
    '''
    Function publishes the setpoint for the PID controller of the yaw angle based on the current setpoint value for the yaw angle
    '''
    msg = Float64()
    msg.data = interface.action_values["yaw"]
    yaw_pid_setpoint_publisher.publish(msg)
    return


def publish_roll_pitch_yawrate_thrust_msg():
    '''
    Function publishes the roll pitch yawrate thrust message.
    The values are the commanded roll and pitch angle as well as the controll efforts determined by the PID controllers
    for v_z and yaw
    '''
    msg = RollPitchYawrateThrust()
    msg.roll = interface.action_values["roll"]
    msg.pitch = interface.action_values["pitch"]
    msg.yaw_rate = interface.yaw_control_effort
    msg.thrust = Vector3()
    msg.thrust.z = interface.v_z_control_effort
    roll_pitch_yawrate_thrust_publisher.publish(msg)
    return 

def process_action(msg):
    '''
    Function triggers the processing procedure whenever a new msg is received from the agent
    '''
    for msg_string in interface.action_values.keys():
        interface.action_values[msg_string] = getattr(msg,msg_string)

    publish_setpoint_v_z()
    publish_setpoint_yaw()
    return  

def reset_action_values(msg):
    '''
    Function handles the reset procedure required to assign the initial values to the actions at the begining of each episode of training.
    '''
    if msg.data == True:
        print("Reset initiated")
        interface.action_values = deepcopy(action_values_start)
        print("New interface.action_values after reset: ", interface.action_values)
    return 

          

if __name__ == '__main__':
    #Init nodes and subscribers
    rospy.init_node(node_name)
    #Set up subscribers
    action_to_interface_subscriber = rospy.Subscriber(action_to_interface_topic[0],action_to_interface_topic[1],process_action)
    reset_simulation_subscriber = rospy.Subscriber(reset_simulation_topic[0],reset_simulation_topic[1],reset_action_values)
    v_z_pid_control_effort_subscriber = rospy.Subscriber(v_z_pid_control_effort_topic[0],v_z_pid_control_effort_topic[1],read_v_z_control_effort)
    yaw_pid_control_effort_subscriber = rospy.Subscriber(yaw_pid_control_effort_topic[0],yaw_pid_control_effort_topic[1],read_yaw_control_effort)
    pose_subscriber = rospy.Subscriber(pose_topic[0],pose_topic[1],read_pose)
    twist_subscriber = rospy.Subscriber(twist_topic[0],twist_topic[1],read_twist)
    #Init rate
    rate = rospy.Rate(publish_hz)

    #commands to be executed as long as node is up
    while not rospy.is_shutdown():
        publish_roll_pitch_yawrate_thrust_msg()     
        rate.sleep()


