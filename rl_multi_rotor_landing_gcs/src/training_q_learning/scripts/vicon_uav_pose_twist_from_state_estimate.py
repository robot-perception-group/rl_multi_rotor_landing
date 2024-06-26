"""This scripts defines a conversion node that converts messages of type roll_pitch_yawrate_thrust to uavpose"""

from uav_msgs.msg import uav_pose
from copy import deepcopy
from geometry_msgs.msg import PoseStamped, TwistStamped
import rospy
from training_q_learning.parameters import Parameters

#Get parameters
node_name='uav_pose_twist_from_state_estimate'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name','iris')
fc_name = rospy.get_param(rospy.get_namespace()+node_name+'/fc_name','fc0')

#Define topics
state_estimate_topic = ('/'+fc_name+'/pose',uav_pose)
uav_pose_topic = ('/'+drone_name+'/pose',PoseStamped)
uav_twist_topic = ('/'+drone_name+'/twist',TwistStamped)

# Script variables
parameters = Parameters()

#Class definition
class UAVPoseFromStateEstimate():
    def __init__(self):
        """Class stores all data that is required to convert roll pitch yawrate thrust to uav_pose"""
        #Define data
        self.uav_pose = PoseStamped()
        self.uav_twist = TwistStamped()
        self.state_estimate = uav_pose()
        
        #Define subscribers
        self.state_estimate_subscriber = rospy.Subscriber(state_estimate_topic[0],state_estimate_topic[1],self.read_state_estimate)

        #Define publishers
        self.uav_pose_publisher = rospy.Publisher(uav_pose_topic[0],uav_pose_topic[1],queue_size=0)
        self.uav_twist_publisher = rospy.Publisher(uav_twist_topic[0],uav_twist_topic[1],queue_size=0)
        return

    def extract_pose_twist_from_state_estimate(self):
        self.uav_pose.header.frame_id = "world"
        self.uav_pose.header.stamp = rospy.Time.now()
        self.uav_pose.pose.position.x = self.state_estimate.position.x
        self.uav_pose.pose.position.y = self.state_estimate.position.y
        self.uav_pose.pose.position.z = self.state_estimate.position.z
        self.uav_pose.pose.orientation.x = self.state_estimate.orientation.x
        self.uav_pose.pose.orientation.y = self.state_estimate.orientation.y
        self.uav_pose.pose.orientation.z = self.state_estimate.orientation.z
        self.uav_pose.pose.orientation.w = self.state_estimate.orientation.w

        self.uav_twist.header.frame_id = "world"
        self.uav_twist.header.stamp = rospy.Time.now()
        self.uav_twist.twist.linear.x = self.state_estimate.velocity.x
        self.uav_twist.twist.linear.y = self.state_estimate.velocity.y
        self.uav_twist.twist.linear.z = self.state_estimate.velocity.z
        self.uav_twist.twist.angular.x = self.state_estimate.angVelocity.x
        self.uav_twist.twist.angular.y = self.state_estimate.angVelocity.y
        self.uav_twist.twist.angular.z = self.state_estimate.angVelocity.z
        return

    def read_state_estimate(self,msg):
        """Function processes the msgs received on the topic"""
        self.state_estimate = deepcopy(msg)
        self.extract_pose_twist_from_state_estimate()
        self.uav_pose_publisher.publish(self.uav_pose)
        self.uav_twist_publisher.publish(self.uav_twist)
        return

if __name__ == '__main__':
    #Init nodes 
    rospy.init_node(node_name)
    uav_pose_from_state_estimate = UAVPoseFromStateEstimate() 
    rospy.loginfo("uav_pose and uav_twist from state estimate conversion node started...")
    rospy.spin()
