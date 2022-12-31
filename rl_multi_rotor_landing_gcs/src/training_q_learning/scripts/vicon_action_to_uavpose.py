"""This scripts defines a conversion node that converts messages of type roll_pitch_yawrate_thrust to uavpose"""

from uav_msgs.msg import uav_pose
from training_q_learning.msg import Action as ActionMsg
from copy import deepcopy
import rospy
from training_q_learning.parameters import Parameters
import numpy as np

#Get parameters
node_name='action_to_uavpose'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name','iris')
fc_name = rospy.get_param(rospy.get_namespace()+node_name+'/fc_name','fc0')

#Define topics
action_topic = ('command/action',ActionMsg)
uav_pose_topic = ('/'+fc_name+'/command',uav_pose)

# Script variables
parameters = Parameters()
M_PI = 3.141592

#Class definition
class ActionToUavPose():
    def __init__(self):
        """Class stores all data that is required to convert roll pitch yawrate thrust to uav_pose"""
        #Define data
        self.action = ActionMsg()
        self.uav_pose = uav_pose()
        
        #Define subscribers
        self.action_subscriber = rospy.Subscriber(action_topic[0],action_topic[1],self.read_action_msg)

        #Define publishers
        self.uav_pose_publisher = rospy.Publisher(uav_pose_topic[0],uav_pose_topic[1],queue_size=0)
        return

    def convert(self):
        """
        Funcion assigns the values of roll, pitch and yaw to the velocity field of the uav pose message. The commanded vertical velocity is
        assigned to the thrust field of the uav_pose message.
        ---
        uav_pose msg:
        Header header
        geometry_msgs/Point position
        geometry_msgs/Point velocity
        geometry_msgs/Quaternion orientation  
        float64[100] covariance
        geometry_msgs/Point angVelocity
        float64 thrust
        int32 flightmode
        geometry_msgs/Point POI
        """
        self.uav_pose.velocity.x = np.rad2deg(self.action.roll)
        self.uav_pose.velocity.y =  np.rad2deg(self.action.pitch)
        self.uav_pose.velocity.z = np.rad2deg(self.action.yaw)
        self.uav_pose.thrust = -self.action.v_z
        self.uav_pose.flightmode = 4
        return

    def read_action_msg(self,msg:ActionMsg):
        """Function processes the msgs received on the topic"""
        self.action = deepcopy(msg)
        self.convert()
        self.uav_pose_publisher.publish(self.uav_pose)
        return

if __name__ == '__main__':
    #Init nodes 
    rospy.init_node(node_name)
    rpyt2uavpose = ActionToUavPose() 
    rospy.loginfo("action to uavpose conversion node started...")
    rospy.spin()
