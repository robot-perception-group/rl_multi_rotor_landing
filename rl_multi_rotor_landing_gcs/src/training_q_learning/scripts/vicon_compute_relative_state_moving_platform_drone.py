'''
This scripts defines a ROS node that  provides information about the moving platform and the drone that are detected by the Vicon system. 
The information for the drone are retrieved from the state estimate that is determined by the drone's flight computer.
The information for the moving platform is directly received from the Vicon system.
NOTE: The frame the data arrives in is "world". Unlike the simulation, where this was an ENU frame, "world" is no a NED frame.


It publishes the relative position, velocity and acceleration on separate topics.
The coordinate frame in which the values are displayed is the stability axes frame
    - Relative position: drone_name/landing_simulation/relative_moving_platform_drone/state/twist/pose
    - Relative velocity: drone_name/landing_simulation/relative_moving_platform_drone/state/twist/twist
    - Relative acceleration: drone_name/landing_simulation/relative_moving_platform_drone/state/twist/acceleration

Furthermore, this node publishes on two different topics the entire movement information required for the landing process for both, 
the drone and the moving platform in stability axes and the world frame.
    - Drone: drone_name/landing_simulation/drone/state
    - Moving platform: drone_name/landing_simulation/moving_platform/state                      
'''

import rospy
from std_msgs.msg import Float64MultiArray
import tf2_ros
from geometry_msgs.msg import TwistStamped, Vector3Stamped, PoseStamped,  Quaternion
from sensor_msgs.msg import Imu
import tf2_geometry_msgs 
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse
from training_q_learning.msg import LandingSimulationObjectState
import numpy as np
from copy import deepcopy

#Parameters
node_name='vicon_compute_relative_state_moving_platform_drone_node'
vicon_relative_state_moving_platform_drone_hz = float(rospy.get_param(rospy.get_namespace()+node_name+'/vicon_relative_state_moving_platform_drone_hz',"10"))
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name','iris')

#Subscriber topics
# drone_pose_topic = ('/vicon/drone/pose',PoseStamped)  #Vicon data
# drone_twist_topic = ('/vicon/drone/twist',TwistStamped) #Vicon data
drone_pose_topic = ('/'+drone_name+'/pose',PoseStamped) #State estimate data, NED frame
drone_twist_topic = ('/'+drone_name+'/twist',TwistStamped) #State estimate data, NED frame
moving_platform_pose_topic = ('/vicon/moving_platform/pose',PoseStamped) #NED frame, eventhough frame_id says "world" (which usually means ENU)
moving_platform_twist_topic = ('/vicon/moving_platform/twist',TwistStamped) #NED frame, eventhough frame_id says "world" (which usually means ENU)

#Publisher topics
relative_vel_state_topic = ('/vicon/relative_moving_platform_drone/state/twist',TwistStamped)
relative_pos_state_topic = ('/vicon/relative_moving_platform_drone/state/pose',PoseStamped)
drone_state_topic = ('/vicon/drone/state',LandingSimulationObjectState)
moving_platform_state_topic = ('/vicon/moving_platform/state',LandingSimulationObjectState)
relative_rpy_topic = ("/vicon/relative_moving_platform_drone/debug_target_frame/roll_pitch_yaw",Float64MultiArray)
drone_rpy_topic = ("/vicon/drone/debug_target_frame/roll_pitch_yaw",Float64MultiArray)
moving_platform_rpy_topic = ("/vicon/moving_platform/debug_target_frame/roll_pitch_yaw",Float64MultiArray)

#Publisher definitions
relative_vel_state_publisher = rospy.Publisher(relative_vel_state_topic[0],relative_vel_state_topic[1],queue_size = 0)
relative_pos_state_publisher = rospy.Publisher(relative_pos_state_topic[0],relative_pos_state_topic[1],queue_size = 0)
relative_rpy_publisher = rospy.Publisher(relative_rpy_topic[0],relative_rpy_topic[1],queue_size=0)
drone_rpy_publisher = rospy.Publisher(drone_rpy_topic[0],drone_rpy_topic[1],queue_size=0)
moving_platform_rpy_publisher = rospy.Publisher(moving_platform_rpy_topic[0],moving_platform_rpy_topic[1],queue_size=0)
drone_state_publisher = rospy.Publisher(drone_state_topic[0],drone_state_topic[1],queue_size = 0)
moving_platform_state_publisher = rospy.Publisher(moving_platform_state_topic[0],moving_platform_state_topic[1],queue_size = 0)

vicon_frame = 'world'
target_frame = drone_name+'/stability_axes'

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


def read_drone_pose(msg):
    """Reads drone pose message and stores the data in storage class."""
    drone_state_original.pose = msg
    return

def read_drone_twist(msg):
    """Reads drone twist message and stores data in stroage class."""
    drone_state_original.twist.twist.linear.vector.x = msg.twist.linear.x
    drone_state_original.twist.twist.linear.vector.y = msg.twist.linear.y
    drone_state_original.twist.twist.linear.vector.z = msg.twist.linear.z
    return

def read_moving_platform_pose(msg):
    """Reads pose message of moving platform and stores data in storage class."""
    moving_platform_state_original.pose = msg
    return

def read_moving_platform_twist(msg):
    """Reads twist message of moving platform and stores data in storage class."""
    moving_platform_state_original.twist.twist.linear.vector.x = msg.twist.linear.x
    moving_platform_state_original.twist.twist.linear.vector.y = msg.twist.linear.y
    moving_platform_state_original.twist.twist.linear.vector.z = msg.twist.linear.z
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
    
    #Determine relative rotation from moving platform to drone using quaterions
    #Initialize array for quaternions
    q_0 = [0,0,0,1]
    q_1 = [0,0,0,1]

    #Create q_0 and q_0_inv
    q_0[0] = moving_platform_state_in_target_frame.pose.pose.orientation.x
    q_0[1] = moving_platform_state_in_target_frame.pose.pose.orientation.y
    q_0[2] = moving_platform_state_in_target_frame.pose.pose.orientation.z
    q_0[3] = moving_platform_state_in_target_frame.pose.pose.orientation.w
    q_0_inv = quaternion_inverse(q_0)
    
    #Create q_1
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
    msg.data = (180/np.pi)*np.array(euler_from_quaternion([q_new.x,q_new.y,q_new.z,q_new.w]))
    relative_rpy_publisher.publish(msg)
    return rel_pose_msg

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
    euler = (180/3.141592)*np.array(euler_from_quaternion([q.x,q.y,q.z,q.w]))
    msg.data = euler
    return msg


if __name__ == '__main__':
    #Init nodes and subscribers
    rospy.init_node(node_name)

    #Define subscribers
    drone_state_subscriber = rospy.Subscriber(drone_pose_topic[0],drone_pose_topic[1],read_drone_pose)
    drone_state_subscriber = rospy.Subscriber(drone_twist_topic[0],drone_twist_topic[1],read_drone_twist)
    moving_platform_state_subscriber = rospy.Subscriber(moving_platform_pose_topic[0],moving_platform_pose_topic[1],read_moving_platform_pose)
    moving_platform_state_subscriber = rospy.Subscriber(moving_platform_twist_topic[0],moving_platform_twist_topic[1],read_moving_platform_twist)

    #Init tf2
    tfBuffer = tf2_ros.Buffer()
    #time.sleep(10)
    listener = tf2_ros.TransformListener(tfBuffer)

    #Init rate
    rate = rospy.Rate(vicon_relative_state_moving_platform_drone_hz)

    #commands to be executed as long as node is up
    while not rospy.is_shutdown():
        rospy.loginfo(vicon_relative_state_moving_platform_drone_hz)

        try:
            #transform drone and moving platform position and velocity in target frame .                                    
            trans_world_to_target_frame = tfBuffer.lookup_transform(target_frame,vicon_frame, rospy.Time())

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

        except (tf2_ros.ExtrapolationException):
            rospy.logwarn('transformation from vicon model frame to drone frame threw an extrapolation error')
            rate.sleep()
            continue
        except (tf2_ros.ConnectivityException):
            rospy.logwarn('transformation from vicon model frame to drone frame threw a connectivity error')
            rate.sleep()
            continue
        except (tf2_ros.LookupException):
            rospy.logwarn('transformation from vicon model frame to drone frame threw a lookup error')
            rate.sleep()
            continue

        #Publish the messages
        rel_vel_msg = compute_relative_vel_msg()
        relative_vel_state_publisher.publish(rel_vel_msg)
        rel_pos_msg = compute_relative_pos_msg()
        relative_pos_state_publisher.publish(rel_pos_msg)
        drone_state_msg = compute_landing_simulation_object_state_msg(drone_state_in_target_frame)
        drone_state_publisher.publish(drone_state_msg)
        drone_rpy_msg = compute_rpy_std_msg(drone_state_in_target_frame.pose.pose.orientation)
        drone_rpy_publisher.publish(drone_rpy_msg)
        moving_platform_msg = compute_landing_simulation_object_state_msg(moving_platform_state_in_target_frame)
        moving_platform_state_publisher.publish(moving_platform_msg)
        moving_platform_rpy_msg = compute_rpy_std_msg(moving_platform_state_in_target_frame.pose.pose.orientation)
        moving_platform_rpy_publisher.publish(moving_platform_rpy_msg)
        rate.sleep()