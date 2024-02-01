'''
This script can be used as an interface to combine to instances of the ROS pid package to form a cascaded pid controller.
Its purpose is to control the relative position and the relative velocity simultaneously in 1D. 
The inner loop controls the relative velocity via the attitude angle associated with the axis of movement. The control effort of the 
inner loop is published to the cascaded pid interface where the data is combined with the control efforts for the other axis 
into the message the attitude controller of the rotorS package can process.
The outer loop controls the relative position.
'''

import rospy
from training_q_learning.msg import ObservationRelativeState as ObservationRelativeStateMsg
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped
from training_q_learning.utils import get_publisher
import numpy as np





#Get parameters
#Get axis identifier
#Parameters
# print("paramter_list = ",rospy.get_param_names())
rospy.init_node('default_name') # will be overwritten by name in launch file
axis_identifier = rospy.get_param(rospy.get_name()+'/axis_identifier') # x,y,z
ol_relative_position_setpoint = float(rospy.get_param(rospy.get_name()+'/relative_position_setpoint','0'))
outer_loop_publish_hz = float(rospy.get_param(rospy.get_name()+'/outer_loop_publish_hz','10'))
inner_loop_publish_hz = float(rospy.get_param(rospy.get_name()+'/inner_loop_publish_hz','20'))
drone_name = rospy.get_param(rospy.get_name()+'/drone_name','iris')   
std_rel_p_x = float(rospy.get_param(rospy.get_name()+'/std_rel_p_x','0'))
std_rel_p_y = float(rospy.get_param(rospy.get_name()+'/std_rel_p_y','0'))
std_rel_p_z = float(rospy.get_param(rospy.get_name()+'/std_rel_p_z','0'))
std_rel_v_x = float(rospy.get_param(rospy.get_name()+'/std_rel_v_x','0'))
std_rel_v_y = float(rospy.get_param(rospy.get_name()+'/std_rel_v_y','0'))
std_rel_v_z = float(rospy.get_param(rospy.get_name()+'/std_rel_v_z','0'))



#Define topic to communicate with attitude controler of the RotorS package and the pid controllers
#Index ol - outer loop, il - inner loop
#Publisher topics
ol_relative_position_setpoint_topic = (rospy.get_name()+'/ol_relative_position/setpoint',Float64)
ol_relative_position_state_topic = (rospy.get_name()+'/ol_relative_position/state',Float64)

il_relative_velocity_setpoint_topic = (rospy.get_name()+'/il_relative_velocity/setpoint',Float64)
il_relative_velocity_state_topic = (rospy.get_name()+'/il_relative_velocity/state',Float64)




#Subscriber topics
#Relative information
relative_pose_topic = ('/'+drone_name+'/landing_simulation/relative_moving_platform_drone/state/pose', PoseStamped)
relative_twist_topic = ('/'+drone_name+'/landing_simulation/relative_moving_platform_drone/state/twist', TwistStamped)
ol_relative_position_control_effort_topic = (rospy.get_name()+'/ol_relative_position/control_effort',Float64)
il_relative_velocity_control_effort_topic = (rospy.get_name()+'/il_relative_velocity/control_effort',Float64)




class CascadedPIDData():
    def __init__(self):
        #Initialize variables
        self.axis_identifier = axis_identifier
        self.ol_relative_position_setpoint = Float64()
        self.ol_relative_position_state = Float64()
        self.ol_relative_position_control_effort = Float64()
        self.il_relative_velocity_setpoint = Float64()
        self.il_relative_velocity_state = Float64()
        self.il_relative_velocity_control_effort = Float64()

        self.relative_information = ObservationRelativeStateMsg()

        #define publishers
        self.ol_relative_position_setpoint_publisher = get_publisher(ol_relative_position_setpoint_topic[0],ol_relative_position_setpoint_topic[1],queue_size = 0)
        self.ol_relative_position_state_publisher = get_publisher(ol_relative_position_state_topic[0],ol_relative_position_state_topic[1],queue_size = 0)
        self.il_relative_velocity_setpoint_publisher = get_publisher(il_relative_velocity_setpoint_topic[0],il_relative_velocity_setpoint_topic[1],queue_size = 0)
        self.il_relative_velocity_state_publisher = get_publisher(il_relative_velocity_state_topic[0],il_relative_velocity_state_topic[1],queue_size = 0)
        
        #define subscribers
        self.ol_relative_position_control_effort_subscriber = rospy.Subscriber(ol_relative_position_control_effort_topic[0],ol_relative_position_control_effort_topic[1],self.read_ol_relative_position_control_effort)
        self.il_relative_velocity_control_effort_subscriber = rospy.Subscriber(il_relative_velocity_control_effort_topic[0],il_relative_velocity_control_effort_topic[1],self.read_il_relative_velocity_control_effort)
        self.relative_pose_subscriber = rospy.Subscriber(relative_pose_topic[0],relative_pose_topic[1],self.read_relative_pose)
        self.relative_twist_subscriber = rospy.Subscriber(relative_twist_topic[0],relative_twist_topic[1],self.read_relative_twist)
        return 
    
    def read_relative_pose(self,msg):
        self.relative_pose = msg
        self.relative_pose.pose.position.x += np.random.normal(loc = 0, scale = std_rel_p_x)
        self.relative_pose.pose.position.y += np.random.normal(loc = 0, scale = std_rel_p_y)
        self.relative_pose.pose.position.z += np.random.normal(loc = 0, scale = std_rel_p_z)
        self.ol_relative_position_state = getattr(self.relative_pose.pose.position,self.axis_identifier)
        return
    
    def read_relative_twist(self,msg):
        self.relative_twist = msg
        self.relative_twist.twist.linear.x += np.random.normal(loc = 0, scale = std_rel_v_x)
        self.relative_twist.twist.linear.y += np.random.normal(loc = 0, scale = std_rel_v_y)
        self.relative_twist.twist.linear.z += np.random.normal(loc = 0, scale = std_rel_v_z)
        self.il_relative_velocity_state = getattr(self.relative_twist.twist.linear,self.axis_identifier)
        return

    # def read_relative_information(self,msg):
    #     self.relative_information = msg
    #     self.ol_relative_position_state = getattr(self.relative_information,'rel_p_'+self.axis_identifier)
    #     self.il_relative_velocity_state = getattr(self.relative_information,'rel_v_'+self.axis_identifier)
    #     return

    def read_ol_relative_position_control_effort(self,msg):
        self.ol_relative_position_control_effort  = msg.data
        return

    def read_il_relative_velocity_control_effort(self,msg):
        self.il_relative_velocity_control_effort  = msg.data
        return

    def publish_outer_loop_data(self,event):
        self.ol_relative_position_setpoint_publisher.publish(self.ol_relative_position_setpoint)
        self.ol_relative_position_state_publisher.publish(self.ol_relative_position_state)
        return

    def publish_inner_loop_data(self,event):
        self.il_relative_velocity_setpoint_publisher.publish(self.ol_relative_position_control_effort)
        self.il_relative_velocity_state_publisher.publish(self.il_relative_velocity_state)
        return        





if __name__ == '__main__':
    #Init nodes and subscribers
    
    cascaded_pid_data = CascadedPIDData()
   

    #commands to be executed as long as node is up
    # print(cascaded_pid_data.relative_information)
    # print("")
    rospy.Timer(rospy.Duration(1/outer_loop_publish_hz),cascaded_pid_data.publish_outer_loop_data)
    rospy.Timer(rospy.Duration(1/inner_loop_publish_hz),cascaded_pid_data.publish_inner_loop_data)
    rospy.spin()
    



    