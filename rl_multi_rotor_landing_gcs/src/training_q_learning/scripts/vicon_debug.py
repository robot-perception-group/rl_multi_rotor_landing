"""This scripts is inteded for debug purposes with varying code snippets"""


from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from std_msgs.msg import Float64MultiArray
from copy import deepcopy
import rospy
import math
import numpy as np

#Get parameters
node_name='vicon_debug_node'


#Define topics for subscribers
drone_pose_enu_topic = ("/vicon/drone/pose_enu",PoseStamped)
drone_pose_ned_topic = ("/copter/pose",PoseStamped)
drone_twist_enu_topic = ("/vicon/drone/twist_enu",TwistStamped)
drone_twist_ned_topic = ("/copter/twist",TwistStamped)

#Define topics for publishers
rpy_state_estimate_topic = ("/debug/rpy_state_estimate",Vector3Stamped)
rpy_vicon_topic = ("/debug/rpy_vicon",Vector3Stamped)

#Class definition
class ViconDebug():
    def __init__(self):
        """Class containing debugging functions and required variables"""
        #Data storage
        self.drone_pose_enu = PoseStamped()
        self.drone_pose_ned_from_enu = PoseStamped()
        self.drone_pose_ned = PoseStamped()
        self.drone_twist_enu = TwistStamped()
        self.drone_twist_ned = TwistStamped()
        self.drone_twist_ned_from_enu = TwistStamped()

        #subscribers
        self.vicon_drone_pose_enu_subscriber = rospy.Subscriber(drone_pose_enu_topic[0],drone_pose_enu_topic[1],self.read_vicon_drone_pose_enu)
        self.vicon_drone_pose_ned_subscriber = rospy.Subscriber(drone_pose_ned_topic[0],drone_pose_ned_topic[1],self.read_vicon_drone_pose_ned)
        self.vicon_drone_twist_enu_subscriber = rospy.Subscriber(drone_twist_enu_topic[0],drone_twist_enu_topic[1],self.read_vicon_drone_twist_enu)
        self.vicon_drone_twist_ned_subscriber = rospy.Subscriber(drone_twist_ned_topic[0],drone_twist_ned_topic[1],self.read_vicon_drone_twist_ned)

        #Define publishers
        self.drone_rpy_state_estimate_publisher = rospy.Publisher(rpy_state_estimate_topic[0],rpy_state_estimate_topic[1],queue_size=1)
        self.drone_rpy_vicon_publisher = rospy.Publisher(rpy_vicon_topic[0],rpy_vicon_topic[1],queue_size=1)

        #Script variables
        self.new_vicon_pose_msg_received = False
        self.new_vicon_twist_msg_received = False
        return

    def read_vicon_drone_pose_enu(self,msg):
        """Reads vicon drone pose message."""
        self.drone_pose_enu = msg
        self.drone_pose_ned_from_enu = self.convert_PoseStamped_from_enu_to_ned(deepcopy(self.drone_pose_enu))
        return
    
    def read_vicon_drone_pose_ned(self,msg):
        """Reads state estimate drone pose message."""
        self.drone_pose_ned = msg
        return
    
    def read_vicon_drone_twist_enu(self,msg):
        """Reads vicon drone twist message."""
        self.drone_twist_enu = msg
        self.drone_twist_ned_from_enu = self.convert_TwistStampedModified_from_enu_to_ned(deepcopy(self.drone_twist_enu))
        return
    
    def read_vicon_drone_twist_ned(self,msg):
        """Reads state estimate drone twist message."""
        self.drone_twist_ned = msg
        return



    def quat2euler_ned(self,quat):
        """Convert quaterion defined in ENU frame to NED frame?"""
        #Switch to hamilton convention
        q = [quat.w,quat.x,quat.y,quat.z]
        q0s = q[0] * q[0]
        q1s = q[1] * q[1]
        q2s = q[2] * q[2]
        q3s = q[3] * q[3]

        R13 = 2.0 * (q[1] * q[3] - q[0] * q[2])
        R11 = q0s + q1s - q2s - q3s
        R12 = 2.0 * (q[1] * q[2] + q[0] * q[3])
        R23 = 2.0 * (q[2] * q[3] + q[0] * q[1])
        R33 = q0s - q1s - q2s + q3s

        rpy = [0,0,0]
        rpy[1] = np.rad2deg(math.asin(-R13)) # pitch always between -pi/2 to pi/2
        rpy[2] = np.rad2deg(math.atan2(R12, R11))
        rpy[0] = np.rad2deg(math.atan2(R23, R33))
        return rpy
    
    def convert_PoseStamped_from_enu_to_ned(self,pose_enu):
        """Convert PoseStamped message from ENU to NED frame."""
        pose_ned = deepcopy(pose_enu)
        pose_ned.header.frame_id = 'world_ned'

        #Convert position by swapping the x,y axes and negating the z-axis
        pose_ned.pose.position.x = pose_enu.pose.position.y
        pose_ned.pose.position.y = pose_enu.pose.position.x
        pose_ned.pose.position.z = -pose_enu.pose.position.z

        # Convert orientation by rotating 90deg around positive z-axis in ENU frame to consider origin of rotation in NED frame.
        # Then swapping the x,y component of quaternion and negating the z-axis acounts for the change in axis from ENU to NED frame.
        # The real part remains unchanged.
        q_orig=[pose_enu.pose.orientation.x,pose_enu.pose.orientation.y,pose_enu.pose.orientation.z,pose_enu.pose.orientation.w]
        q_converted=[0.,0.,0.,0.]
        q_converted[0]=q_orig[1]
        q_converted[1]=q_orig[0]
        q_converted[2]=-q_orig[2]
        q_converted[3]=q_orig[3]

        pose_ned.pose.orientation.x = q_converted[0]
        pose_ned.pose.orientation.y = q_converted[1]
        pose_ned.pose.orientation.z = q_converted[2]
        pose_ned.pose.orientation.w = q_converted[3]
        rpy = self.quat2euler_ned(pose_ned.pose.orientation)
        return pose_ned

    def convert_TwistStampedModified_from_enu_to_ned(self,twist_enu):
        """Converts twist stamped message from ENU frame to NED frame."""
        twist_ned = deepcopy(twist_enu)
        twist_ned.header.frame_id = 'world_ned'

        #Convert linear twist by swapping the x,y axes and negating the z-axis
        twist_ned.twist.linear.x = twist_enu.twist.linear.y
        twist_ned.twist.linear.y = twist_enu.twist.linear.x
        twist_ned.twist.linear.z = -twist_enu.twist.linear.z

        #Convert angular twist by swapping the x,y component and negating the z-axis.
        twist_ned.twist.angular.x = twist_enu.twist.angular.y
        twist_ned.twist.angular.y = twist_enu.twist.angular.x
        twist_ned.twist.angular.z = -twist_enu.twist.angular.z
        return twist_ned
        

    def publish_data(self):
        """
        Publish debug data.
        """
        rpy_state_estimate = self.quat2euler_ned(self.drone_pose_ned.pose.orientation)
        rpy_vicon = self.quat2euler_ned(self.drone_pose_ned_from_enu.pose.orientation)
        
        msg_rpy_state_estimate = Vector3Stamped()
        msg_rpy_state_estimate.header.stamp = self.drone_pose_ned.header.stamp
        msg_rpy_state_estimate.vector.x = rpy_state_estimate[0]
        msg_rpy_state_estimate.vector.y = rpy_state_estimate[1]
        msg_rpy_state_estimate.vector.z = rpy_state_estimate[2]
        
        msg_rpy_vicon = Vector3Stamped()
        msg_rpy_vicon.header.stamp = self.drone_pose_ned_from_enu.header.stamp
        msg_rpy_vicon.vector.x = rpy_vicon[0]
        msg_rpy_vicon.vector.y = rpy_vicon[1]
        msg_rpy_vicon.vector.z = rpy_vicon[2]

        self.drone_rpy_state_estimate_publisher.publish(msg_rpy_state_estimate)
        self.drone_rpy_vicon_publisher.publish(msg_rpy_vicon)
        return

if __name__ == '__main__':
    #Init nodes 
    rospy.init_node(node_name)
    debug_node = ViconDebug() 
    rospy.loginfo("vicon debug node started...")
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        debug_node.publish_data()
        rate.sleep()
