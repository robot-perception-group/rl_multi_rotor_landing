"""This scripts reads vicon data and redistributes it to required topics to run the landing algorithm. It performs a transformation from ENU to NED frame."""


from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from copy import deepcopy
import rospy
from training_q_learning.msg import LandingSimulationObjectState
import math


#Get parameters
node_name='vicon_interface'
vicon_interface_publish_hz = float(rospy.get_param(rospy.get_namespace()+node_name+'/vicon_interface_publish_hz',"10"))
vicon_drone_id = rospy.get_param(rospy.get_namespace()+node_name+'/vicon_drone_id','drone')
vicon_mp_id = rospy.get_param(rospy.get_namespace()+node_name+'/vicon_mp_id','moving_platform')
fc_name = rospy.get_param(rospy.get_namespace()+node_name+'/fc_name','fc0')

#Define topics
#vicon topics
vicon_drone_pos_topic = ('/vicon/drone/pose_enu',PoseStamped)
vicon_drone_vel_topic = ('/vicon/drone/twist_enu',TwistStamped)
vicon_mp_pos_topic = ('/vicon/moving_platform/pose_enu_filtered',PoseStamped)
vicon_mp_vel_topic = ('/vicon/moving_platform/twist_enu_filtered',TwistStamped)

#mp transformation topics
vicon_mp_pos_ned_topic = ('/vicon/moving_platform/pose',PoseStamped)
vicon_mp_vel_ned_topic =('/vicon/moving_platform/twist',TwistStamped)

#fc topics
auxposition_topic = ('/'+fc_name+'/auxposition',Vector3Stamped)
auxvelocity_topic = ('/'+fc_name+'/auxvelocity',Vector3Stamped)

#real world topics
drone_state_topic = ('/vicon/drone/world_frame/state',LandingSimulationObjectState)
mp_state_topic = ('/vicon/moving_platform/world_frame/state',LandingSimulationObjectState)


#Class definition
class ViconInterface():
    def __init__(self):
        """Function stores the pose and twist information together with the reference frame."""
        #Data storage
        self.drone_world = LandingSimulationObjectState()
        self.drone_world.header.frame_id = 'world'
        self.mp_world = LandingSimulationObjectState()
        self.mp_world.header.frame_id = 'world'

        self.drone_ned = LandingSimulationObjectState()
        self.drone_ned.header.frame_id = 'world_ned'
        self.mp_ned = LandingSimulationObjectState()
        self.mp_ned.header.frame_id = 'world_ned'

        #subscribers
        self.vicon_drone_pos_subscriber = rospy.Subscriber(vicon_drone_pos_topic[0],vicon_drone_pos_topic[1],self.read_vicon_drone_pose)
        self.vicon_drone_vel_subscriber = rospy.Subscriber(vicon_drone_vel_topic[0],vicon_drone_vel_topic[1],self.read_vicon_drone_vel)
        self.vicon_mp_pos_subscriber = rospy.Subscriber(vicon_mp_pos_topic[0],vicon_mp_pos_topic[1],self.read_vicon_mp_pose)
        self.vicon_mp_vel_subscriber = rospy.Subscriber(vicon_mp_vel_topic[0],vicon_mp_vel_topic[1],self.read_vicon_mp_vel)

        #publishers
        self.auxposition_publisher = rospy.Publisher(auxposition_topic[0],auxposition_topic[1],queue_size = 0)
        self.auxvelocity_publisher = rospy.Publisher(auxvelocity_topic[0],auxvelocity_topic[1],queue_size = 0)
        self.drone_state_publisher = rospy.Publisher(drone_state_topic[0],drone_state_topic[1],queue_size = 0)
        self.mp_state_publisher = rospy.Publisher(mp_state_topic[0],mp_state_topic[1],queue_size = 0)
        self.mp_pos_ned_publisher = rospy.Publisher(vicon_mp_pos_ned_topic[0],vicon_mp_pos_ned_topic[1],queue_size = 0)
        self.mp_vel_ned_publisher = rospy.Publisher(vicon_mp_vel_ned_topic[0],vicon_mp_vel_ned_topic[1],queue_size = 0)

        #Script variables
        self.new_vicon_pose_msg_received = False
        self.new_vicon_twist_msg_received = False
        return

    def read_vicon_drone_pose(self,msg):
        """Reads vicon drone pose message, performs the frame conversion from ENU to NED and stores data in storage class."""
        self.drone_world.pose = msg
        self.drone_world.header.frame_id = 'world'
        self.drone_ned.pose = self.convert_PoseStamped_from_enu_to_ned(deepcopy(self.drone_world.pose))
        self.new_vicon_pose_msg_received = True
        return

    def read_vicon_drone_vel(self,msg):
        """Reads vicon drone pose message, performs the frame conversion from ENU to NED and stores data in storage class."""
        self.drone_world.twist.header = deepcopy(msg.header)
        self.drone_world.twist.header.frame_id = 'world'
        self.drone_world.twist.twist.linear.vector.x = msg.twist.linear.x
        self.drone_world.twist.twist.linear.vector.y = msg.twist.linear.y
        self.drone_world.twist.twist.linear.vector.z = msg.twist.linear.z
        self.drone_ned.twist = self.convert_TwistStampedModified_from_enu_to_ned(deepcopy(self.drone_world.twist))
        self.new_vicon_twist_msg_received = True
        return

    def read_vicon_mp_pose(self,msg):
        """Reads vicon moving platform pose message, performs the frame conversion from ENU to NED and stores data in storage class."""
        self.mp_world.pose = msg
        self.mp_world.header.frame_id = 'world'
        self.mp_ned.pose = self.convert_PoseStamped_from_enu_to_ned(deepcopy(self.mp_world.pose))
        return

    def read_vicon_mp_vel(self,msg):
        """Reads vicon drone vel message, performs the frame conversion from ENU to NED and stores data in storage class."""
        self.mp_world.twist.header = deepcopy(msg.header)
        self.mp_world.twist.header.frame_id = 'world'
        self.mp_world.twist.twist.linear.vector.x = msg.twist.linear.x
        self.mp_world.twist.twist.linear.vector.y = msg.twist.linear.y
        self.mp_world.twist.twist.linear.vector.z = msg.twist.linear.z
        self.mp_ned.twist = self.convert_TwistStampedModified_from_enu_to_ned(deepcopy(self.mp_world.twist))
        return

    def quat2euler_ned(self,quat):
        """Convert quaterion defined in ENU frame to NED frame"""
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
        rpy[1] = math.asin(-R13) # pitch always between -pi/2 to pi/2
        rpy[2] = math.atan2(R12, R11)
        rpy[0] = math.atan2(R23, R33)
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
        twist_ned.twist.linear.vector.x = twist_enu.twist.linear.vector.y
        twist_ned.twist.linear.vector.y = twist_enu.twist.linear.vector.x
        twist_ned.twist.linear.vector.z = -twist_enu.twist.linear.vector.z

        #Convert angular twist by swapping the x,y component and negating the z-axis.
        twist_ned.twist.angular.vector.x = twist_enu.twist.angular.vector.y
        twist_ned.twist.angular.vector.y = twist_enu.twist.angular.vector.x
        twist_ned.twist.angular.vector.z = -twist_enu.twist.angular.vector.z
        return twist_ned
        

    def publish_data(self):
        """Publishes auxposition, auxvelocity in NED frame only when a new message was received from the Vicon system.
        Also publishes drone state and mp state in world frame (ENU).
        Also publishes the moving platform position and velocity in NED frame, although the message frame_id says 
        "world". This is necessary so that other nodes that were originally designed to work only in 
        simulation (where ENU frame is always used) can process the data.
        """
        #publish auxposition
        msg_auxposition = Vector3Stamped()
        msg_auxposition.header.stamp = rospy.Time.now()
        msg_auxposition.header.frame_id = self.drone_ned.header.frame_id
        msg_auxposition.vector.x = self.drone_ned.pose.pose.position.x
        msg_auxposition.vector.y = self.drone_ned.pose.pose.position.y
        msg_auxposition.vector.z = self.drone_ned.pose.pose.position.z
        if self.new_vicon_pose_msg_received:
            self.auxposition_publisher.publish(msg_auxposition)
            self.new_vicon_pose_msg_received = False

        #publish auxvelocity
        msg_auxvelocity = Vector3Stamped()
        msg_auxvelocity.header.stamp = rospy.Time.now()
        msg_auxvelocity.header.frame_id = self.drone_ned.header.frame_id
        msg_auxvelocity.vector.x = self.drone_ned.twist.twist.linear.vector.x 
        msg_auxvelocity.vector.y = self.drone_ned.twist.twist.linear.vector.y 
        msg_auxvelocity.vector.z = self.drone_ned.twist.twist.linear.vector.z
        if self.new_vicon_twist_msg_received:
            self.auxvelocity_publisher.publish(msg_auxvelocity)
            self.new_vicon_twist_msg_received = False

        #publish world drone and mp state in world frame
        self.drone_state_publisher.publish(self.drone_world)
        self.mp_state_publisher.publish(self.mp_world)

        #Publish the mp_pos data
        mp_pos_msg = PoseStamped()
        mp_pos_msg.header.stamp = rospy.Time.now()
        mp_pos_msg.header.frame_id = "world" #THIS IS NECESSARY FOR THE OTHER NODES TO WORK EVEN THOUGH IT IS ACTUALLY NED FRAME
        mp_pos_msg.pose = deepcopy(self.mp_ned.pose.pose)
        self.mp_pos_ned_publisher.publish(mp_pos_msg)

        #Publish the mp_vel data
        mp_vel_msg = TwistStamped()
        mp_vel_msg.header.stamp = rospy.Time.now()
        mp_vel_msg.header.frame_id = "world" #THIS IS NECESSARY FOR THE OTHER NODES TO WORK EVEN THOUGH IT IS ACTUALLY NED FRAME
        mp_vel_msg.twist.linear.x = self.mp_ned.twist.twist.linear.vector.x 
        mp_vel_msg.twist.linear.y = self.mp_ned.twist.twist.linear.vector.y
        mp_vel_msg.twist.linear.z = self.mp_ned.twist.twist.linear.vector.z 
        mp_vel_msg.twist.angular.x = self.mp_ned.twist.twist.angular.vector.x 
        mp_vel_msg.twist.angular.y = self.mp_ned.twist.twist.angular.vector.y
        mp_vel_msg.twist.angular.z = self.mp_ned.twist.twist.angular.vector.z 
        self.mp_vel_ned_publisher.publish(mp_vel_msg)
        return

if __name__ == '__main__':
    #Init nodes 
    rospy.init_node(node_name)
    vicon_interface = ViconInterface() 
    rospy.loginfo("vicon_interface node started...")
    rate = rospy.Rate(vicon_interface_publish_hz)
    while not rospy.is_shutdown():
        vicon_interface.publish_data()
        rate.sleep()
