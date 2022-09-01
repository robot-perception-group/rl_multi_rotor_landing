#!/usr/bin/env python  
import rospy
from std_msgs.msg import Float64, Float64MultiArray
import tf2_ros
from geometry_msgs.msg import TwistStamped, Vector3Stamped, PoseStamped,  Quaternion
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
import tf2_geometry_msgs 
from tf.transformations import euler_from_quaternion, quaternion_multiply
from training_q_learning.msg import LandingSimulationObjectState
import numpy as np
from copy import deepcopy

M_PI = 3.14159265

'''
This scripts defines a ROS node that  provides information about the moving platform and the drone that are simulated in Gazebo. 

It publishes the relative position, velocity and acceleration on separate topics.
The coordinate frame in which the values are displayed is the stability axis (stability axis frame is called "stability axis")
    - Relative position: drone_name/landing_simulation/relative_moving_platform_drone/state/twist/pose)
    - Relative velocity: drone_name/landing_simulation/relative_moving_platform_drone/state/twist/twist)
    - Relative acceleration: drone_name/landing_simulation/relative_moving_platform_drone/state/twist/acceleration)

Furthermore, this node publishes on two different topics the entire movement information required for the landing process for both, 
the drone and the moving platform.
    - Drone: drone_name/landing_simulation/drone/state
    - Moving platform: drone_name/landing_simulation/moving_platform/state

'''



#Parameters
node_name='compute_relative_state_moving_platform_drone_node'
relative_state_moving_platform_drone_hz = float(rospy.get_param(rospy.get_namespace()+node_name+'/relative_state_moving_platform_drone_hz',"10"))
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name','iris')


#Topic and frame definitions
gazebo_model_states_topic = ('/gazebo/model_states',ModelStates)
drone_imu_topic = ('imu',Imu)
moving_platform_imu_topic = ('/moving_platform/imu',Imu)
relative_vel_state_topic = ('landing_simulation/relative_moving_platform_drone/state/twist',TwistStamped)
relative_pos_state_topic = ('landing_simulation/relative_moving_platform_drone/state/pose',PoseStamped)
relative_acc_state_topic = ('landing_simulation/relative_moving_platform_drone/state/acceleration',Imu)
drone_state_topic = ('landing_simulation/drone/state',LandingSimulationObjectState)
moving_platform_state_topic = ('landing_simulation/moving_platform/state',LandingSimulationObjectState)
drone_state_world_topic = ('landing_simulation/world_frame/drone/state',LandingSimulationObjectState)
moving_platform_state_world_topic = ('landing_simulation/world_frame/moving_platform/state',LandingSimulationObjectState)
relative_rpy_topic = ("landing_simulation/relative_moving_platform_drone/debug_target_frame/roll_pitch_yaw",Float64MultiArray)
drone_rpy_topic = ("landing_simulation/drone/debug_target_frame/roll_pitch_yaw",Float64MultiArray)
moving_platform_rpy_topic = ("landing_simulation/moving_platform/debug_target_frame/roll_pitch_yaw",Float64MultiArray)



#Publisher definitions
relative_vel_state_publisher = rospy.Publisher(relative_vel_state_topic[0],relative_vel_state_topic[1],queue_size = 0)
relative_pos_state_publisher = rospy.Publisher(relative_pos_state_topic[0],relative_pos_state_topic[1],queue_size = 0)
relative_rpy_publisher = rospy.Publisher(relative_rpy_topic[0],relative_rpy_topic[1],queue_size=0)
drone_rpy_publisher = rospy.Publisher(drone_rpy_topic[0],drone_rpy_topic[1],queue_size=0)
moving_platform_rpy_publisher = rospy.Publisher(moving_platform_rpy_topic[0],moving_platform_rpy_topic[1],queue_size=0)
### Uncomment the following line when IMUs are used
#relative_acc_state_publisher = rospy.Publisher(relative_acc_state_topic[0],relative_acc_state_topic[1],queue_size = 0)



drone_state_publisher = rospy.Publisher(drone_state_topic[0],drone_state_topic[1],queue_size = 0)
moving_platform_state_publisher = rospy.Publisher(moving_platform_state_topic[0],moving_platform_state_topic[1],queue_size = 0)
drone_state_world_publisher = rospy.Publisher(drone_state_world_topic[0],drone_state_world_topic[1],queue_size = 0)
moving_platform_state_world_publisher = rospy.Publisher(moving_platform_state_world_topic[0],moving_platform_state_world_topic[1],queue_size = 0)

gazebo_model_state_frame = 'world'
target_frame = drone_name+'/stability_axes'


class OrientationState():
    def __init__(self):
        """Simple class for storing euler angles."""
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

#Class definition
class PoseTwistAccelerationState():
    def __init__(self):
        """Function stores the pose and twist information together with the reference frame."""
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'world'

        self.twist = TwistStamped()
        self.twist.header.frame_id = 'world'
        self.twist.twist.linear = Vector3Stamped()
        self.twist.twist.angular = Vector3Stamped()

        self.linear_acceleration = Vector3Stamped()
        self.linear_acceleration.header.frame_id = 'world'
        return



#Class initialization
drone_state_original = PoseTwistAccelerationState()
moving_platform_state_original = PoseTwistAccelerationState()

drone_state_in_target_frame = PoseTwistAccelerationState()
moving_platform_state_in_target_frame = PoseTwistAccelerationState()

drone_orientation_in_world_frame = PoseTwistAccelerationState()
moving_platform_orientation_in_world_frame =PoseTwistAccelerationState()



def read_gazebo_model_states(msg):
    """ Function reads gazebo model states messages and saves the position and velocity data to the class instances for
    the moving platform and the drone
    """
   

    model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
    object_coordinates_moving_platform = model_coordinates("moving_platform",gazebo_model_state_frame)
    object_coordinates_drone= model_coordinates(drone_name,gazebo_model_state_frame)

    moving_platform_state_original.pose.header.frame_id = gazebo_model_state_frame
    moving_platform_state_original.pose.pose = object_coordinates_moving_platform.pose

    moving_platform_state_original.twist.header.frame_id = gazebo_model_state_frame
    moving_platform_state_original.twist.twist.linear.vector = object_coordinates_moving_platform.twist.linear
    moving_platform_state_original.twist.twist.angular.vector = object_coordinates_moving_platform.twist.angular

    drone_state_original.pose.header.frame_id = gazebo_model_state_frame
    drone_state_original.pose.pose = object_coordinates_drone.pose

    drone_state_original.twist.header.frame_id = gazebo_model_state_frame
    drone_state_original.twist.twist.linear.vector = object_coordinates_drone.twist.linear
    drone_state_original.twist.twist.angular.vector = object_coordinates_drone.twist.angular  
    return   


def read_drone_imu(msg):
    """ Function reads drone imu message and saves it to data class of drone.
    """
    drone_state_original.linear_acceleration.header.frame_id = msg.header.frame_id
    drone_state_original.linear_acceleration.vector.x = msg.linear_acceleration.x
    drone_state_original.linear_acceleration.vector.y = msg.linear_acceleration.y
    drone_state_original.linear_acceleration.vector.z = msg.linear_acceleration.z
    return

def read_moving_platform_imu(msg):
    """ Function reads moving platform imu message and saves it to data class of moving platform
    """
    moving_platform_state_original.linear_acceleration.header.frame_id = msg.header.frame_id
    moving_platform_state_original.linear_acceleration.vector.x = msg.linear_acceleration.x
    moving_platform_state_original.linear_acceleration.vector.y = msg.linear_acceleration.y
    moving_platform_state_original.linear_acceleration.vector.z = msg.linear_acceleration.z
    return

def compute_relative_vel_msg():
    '''
    Function computes the relative velocity vector between the moving platform and the drone.
    '''

    rel_vel_msg = TwistStamped()
    rel_vel_msg.header.stamp = rospy.Time.now()
    rel_vel_msg.header.frame_id = target_frame
    rel_vel_msg.twist.linear.x = moving_platform_state_in_target_frame.twist.twist.linear.vector.x-drone_state_in_target_frame.twist.twist.linear.vector.x
    rel_vel_msg.twist.linear.y = moving_platform_state_in_target_frame.twist.twist.linear.vector.y-drone_state_in_target_frame.twist.twist.linear.vector.y
    rel_vel_msg.twist.linear.z = moving_platform_state_in_target_frame.twist.twist.linear.vector.z-drone_state_in_target_frame.twist.twist.linear.vector.z

    rel_vel_msg.twist.angular.x = moving_platform_state_in_target_frame.twist.twist.angular.vector.x-drone_state_in_target_frame.twist.twist.angular.vector.x
    rel_vel_msg.twist.angular.y = moving_platform_state_in_target_frame.twist.twist.angular.vector.y-drone_state_in_target_frame.twist.twist.angular.vector.y
    rel_vel_msg.twist.angular.z = moving_platform_state_in_target_frame.twist.twist.angular.vector.z-drone_state_in_target_frame.twist.twist.angular.vector.z    
    return rel_vel_msg

def compute_relative_pos_msg():
    '''
    Function computes the relative position vector between the moving platform and the drone.
    '''
    p_x = moving_platform_state_in_target_frame.pose.pose.position.x-drone_state_in_target_frame.pose.pose.position.x
    p_y = moving_platform_state_in_target_frame.pose.pose.position.y-drone_state_in_target_frame.pose.pose.position.y
    p_z = moving_platform_state_in_target_frame.pose.pose.position.z-drone_state_in_target_frame.pose.pose.position.z
    
    #Initialize array for quaternions
    q_0_inv = [0,0,0,1]
    q_1 = [0,0,0,1]

    #Create inverse of q_1 by making the real component negative
    q_0_inv[0] = moving_platform_state_in_target_frame.pose.pose.orientation.x
    q_0_inv[1] = moving_platform_state_in_target_frame.pose.pose.orientation.y
    q_0_inv[2] = moving_platform_state_in_target_frame.pose.pose.orientation.z
    q_0_inv[3] = -moving_platform_state_in_target_frame.pose.pose.orientation.w

    q_1[0] = drone_state_in_target_frame.pose.pose.orientation.x
    q_1[1] = drone_state_in_target_frame.pose.pose.orientation.y
    q_1[2] = drone_state_in_target_frame.pose.pose.orientation.z
    q_1[3] = drone_state_in_target_frame.pose.pose.orientation.w

    #Computate rotation quaternion which yields q_1 = q_r*q_0
    q_r = quaternion_multiply(q_1,q_0_inv)
    q_new = Quaternion()
    q_new.x = q_r[0]
    q_new.y = q_r[1]
    q_new.z = q_r[2]
    q_new.w = q_r[3]

    
    rel_pose_msg = PoseStamped()
    rel_pose_msg.header.frame_id = target_frame
    rel_pose_msg.header.stamp = rospy.Time.now()
    rel_pose_msg.pose.position.x = p_x
    rel_pose_msg.pose.position.y = p_y
    rel_pose_msg.pose.position.z = p_z
    rel_pose_msg.pose.orientation = q_new

    
    
    msg = Float64MultiArray()
    msg.data = (180/M_PI)*np.array(euler_from_quaternion([q_new.x,q_new.y,q_new.z,q_new.w]))
    relative_rpy_publisher.publish(msg)
    return rel_pose_msg

def compute_relative_acc_msg():
    '''
    Function computes the relative acceleration vector between the moving platform and the drone.
    '''
    a_x = moving_platform_state_in_target_frame.linear_acceleration.vector.x-drone_state_in_target_frame.linear_acceleration.vector.x
    a_y = moving_platform_state_in_target_frame.linear_acceleration.vector.y-drone_state_in_target_frame.linear_acceleration.vector.y
    a_z = moving_platform_state_in_target_frame.linear_acceleration.vector.z-drone_state_in_target_frame.linear_acceleration.vector.z
    rel_acc_msg = Imu()
    rel_acc_msg.header.stamp = rospy.Time.now()
    rel_acc_msg.header.frame_id = target_frame
    rel_acc_msg.linear_acceleration.x = a_x
    rel_acc_msg.linear_acceleration.y = a_y
    rel_acc_msg.linear_acceleration.z = a_z
    return rel_acc_msg


def compute_landing_simulation_object_state_msg(landing_simulation_object):
    '''
    Function takes as input an instance of the class LandingSimulationObject and produces a ROS msg
    based on the values that are currently stored in the class. It also performs a check if all
    sensor values are defined in the same coordinate frame.
    '''
    landing_object_state_msg = LandingSimulationObjectState()
    landing_object_state_msg.twist.twist.linear = Vector3Stamped()
    landing_object_state_msg.twist.twist.angular = Vector3Stamped()

    landing_object_state_msg.header.stamp = rospy.Time.now()
    #Check if all coordinate frames are the same
    ### Use this line if IMUs are used
    #if landing_simulation_object.pose.header.frame_id == landing_simulation_object.twist.header.frame_id and landing_simulation_object.pose.header.frame_id == landing_simulation_object.linear_acceleration.header.frame_id:
    if landing_simulation_object.pose.header.frame_id == landing_simulation_object.twist.header.frame_id:
        landing_object_state_msg.header.frame_id = landing_simulation_object.pose.header.frame_id
    else:
        rospy.logwarn("Coordinate frames are not the same for position, velocity and acceleration.")
        rospy.logwarn('Pose frame:')
        rospy.logwarn(landing_simulation_object.pose.header.frame_id)
        rospy.logwarn('Twist frame:')
        rospy.logwarn(landing_simulation_object.twist.header.frame_id)
        rospy.logwarn('Linear acceleration frame:')
        rospy.logwarn(landing_simulation_object.linear_acceleration.header.frame_id)
    landing_object_state_msg.pose = deepcopy(landing_simulation_object.pose)
    landing_object_state_msg.twist = deepcopy(landing_simulation_object.twist)
    landing_object_state_msg.linear_acceleration = deepcopy(landing_simulation_object.linear_acceleration)
    return landing_object_state_msg

def compute_rpy_std_msg(q):
    msg = Float64MultiArray()
    msg.data = euler_from_quaternion([q.x,q.y,q.z,q.w])
    return msg


if __name__ == '__main__':
    #Init nodes and subscribers
    rospy.init_node(node_name)

    gazebo_model_states_subscriber = rospy.Subscriber(gazebo_model_states_topic[0],gazebo_model_states_topic[1],read_gazebo_model_states)
    ### Uncomment the following lines when IMUs are used
    # drone_imu_subscriber = rospy.Subscriber(drone_imu_topic[0],drone_imu_topic[1],read_drone_imu)
    # moving_platform_imu_subscriber = rospy.Subscriber(moving_platform_imu_topic[0],moving_platform_imu_topic[1],read_moving_platform_imu)

    #Init tf2
    tfBuffer = tf2_ros.Buffer()
    #time.sleep(10)
    listener = tf2_ros.TransformListener(tfBuffer)

    #Init rate
    rate = rospy.Rate(relative_state_moving_platform_drone_hz)

    #commands to be executed as long as node is up
    while not rospy.is_shutdown():
        rospy.loginfo(relative_state_moving_platform_drone_hz)
        ### Uncomment the following lines when IMUs are used
        # try:
        #     #transform drone acceleration in target frame 
        #     trans_drone_imu_to_target_frame = tfBuffer.lookup_transform(target_frame,drone_state_original.linear_acceleration.header.frame_id ,rospy.Time())
        #     drone_state_in_target_frame.linear_acceleration = tf2_geometry_msgs.do_transform_vector3(drone_state_original.linear_acceleration,trans_drone_imu_to_target_frame)
        #     drone_state_in_target_frame.linear_acceleration.header.frame_id = target_frame

            
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException): 
        #     rospy.logwarn('error in transformation of acceleration (drone_imu_to_odom)')
        #     #raise
        #     rate.sleep()
        #     continue

        
        ### Uncomment the following lines when IMUs are used
        # try:
        #     #transform moving platform acceleration in target frame                                 
        #     trans_moving_platform_IMU_link_to_target_frame = tfBuffer.lookup_transform(target_frame,moving_platform_state_original.linear_acceleration.header.frame_id, rospy.Time())
        #     moving_platform_state_in_target_frame.linear_acceleration = tf2_geometry_msgs.do_transform_vector3(moving_platform_state_original.linear_acceleration,trans_moving_platform_IMU_link_to_target_frame)
        #     moving_platform_state_in_target_frame.linear_acceleration.header.frame_id = target_frame

        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.logwarn('transformation of drone or trans_moving_platform_IMU_link_to_target_frame threw an error')
        #     rate.sleep()
        #     continue

        try:
            #transform drone and moving platform position and velocity in target frame .                                    
            trans_world_to_target_frame = tfBuffer.lookup_transform(target_frame,gazebo_model_state_frame, rospy.Time())

            moving_platform_state_in_target_frame.pose = tf2_geometry_msgs.do_transform_pose(moving_platform_state_original.pose,trans_world_to_target_frame)
            moving_platform_state_in_target_frame.pose.header.frame_id = target_frame

            moving_platform_state_in_target_frame.twist.twist.linear = tf2_geometry_msgs.do_transform_vector3(moving_platform_state_original.twist.twist.linear,trans_world_to_target_frame)
            moving_platform_state_in_target_frame.twist.twist.angular = tf2_geometry_msgs.do_transform_vector3(moving_platform_state_original.twist.twist.angular,trans_world_to_target_frame)
            moving_platform_state_in_target_frame.twist.header.frame_id = target_frame

            
            drone_state_in_target_frame.pose = tf2_geometry_msgs.do_transform_pose(drone_state_original.pose,trans_world_to_target_frame)
            drone_state_in_target_frame.pose.header.frame_id = target_frame


            drone_state_in_target_frame.twist.twist.linear = tf2_geometry_msgs.do_transform_vector3(drone_state_original.twist.twist.linear,trans_world_to_target_frame)
            drone_state_in_target_frame.twist.twist.angular = tf2_geometry_msgs.do_transform_vector3(drone_state_original.twist.twist.angular,trans_world_to_target_frame)
            drone_state_in_target_frame.twist.header.frame_id = target_frame

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('transformation from gazebo model frame to drone frame threw an error')
            rate.sleep()
            continue

        #Publish the messages
        rel_vel_msg = compute_relative_vel_msg()
        relative_vel_state_publisher.publish(rel_vel_msg)
        rel_pos_msg = compute_relative_pos_msg()
        relative_pos_state_publisher.publish(rel_pos_msg)
        ### Uncomment the following lines when IMUs are used
        # rel_acc_msg = compute_relative_acc_msg()
        # relative_acc_state_publisher.publish(rel_acc_msg)
        drone_state_msg = compute_landing_simulation_object_state_msg(drone_state_in_target_frame)
        drone_state_publisher.publish(drone_state_msg)
        drone_rpy_msg = compute_rpy_std_msg(drone_state_in_target_frame.pose.pose.orientation)
        drone_rpy_publisher.publish(drone_rpy_msg)
        drone_state_world_msg = compute_landing_simulation_object_state_msg(drone_state_original)
        drone_state_world_publisher.publish(drone_state_world_msg)
        moving_platform_msg = compute_landing_simulation_object_state_msg(moving_platform_state_in_target_frame)
        moving_platform_state_publisher.publish(moving_platform_msg)
        moving_platform_rpy_msg = compute_rpy_std_msg(moving_platform_state_in_target_frame.pose.pose.orientation)
        moving_platform_rpy_publisher.publish(moving_platform_rpy_msg)
        moving_platform_world_msg = compute_landing_simulation_object_state_msg(moving_platform_state_original)
        moving_platform_state_world_publisher.publish(moving_platform_world_msg)
        rate.sleep()