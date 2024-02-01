'''
Script provides an interface in which the control efforts (accel) from the x,y,z cascaded pid loops is combined in a 
RollPitchYawrateThrist message and published.
Also this script provides the interface for the yaw control single pid control node. 
'''

import rospy
from mav_msgs.msg import RollPitchYawrateThrust
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from training_q_learning.utils import get_publisher
from training_q_learning.msg import ObservationRelativeState as ObservationRelativeStateMsg
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_inverse
from training_q_learning.msg import Action as ActionMsg
from librepilot.msg import TransmitterInfo




#Parameters
rospy.init_node('default') # node name will be overwritten by node name specified in launch file
drone_name = rospy.get_param(rospy.get_name()+'/drone_name','hummingbird')   
publish_hz = float(rospy.get_param(rospy.get_name()+'/publish_hz','10'))

#define subscriber topics
#Controller related subscription topics
x_relative_velocity_control_effort_topic = ('/'+drone_name+'/vicon_cascaded_pid_x/il_relative_velocity/control_effort',Float64)
y_relative_velocity_control_effort_topic = ('/'+drone_name+'/vicon_cascaded_pid_y/il_relative_velocity/control_effort',Float64)

yaw_setpoint_topic = ('/'+drone_name+'/vicon_cascaded_pid_interface/yaw/setpoint',Float64)
v_z_setpoint_topic = ('/'+drone_name+'/vicon_cascaded_pid_interface/v_z/setpoint',Float64)


# z_relative_velocity_control_effort_topic = ('/'+drone_name+'/cascaded_pid_z/il_relative_velocity/control_effort',Float64)
# yaw_control_effort_topic = ('/'+drone_name+'/cascaded_pid_interface/yaw/control_effort',Float64)
fc_name = rospy.get_param(rospy.get_name()+'/fc_name','fc0')
topic_prefix = '/'+drone_name+'/'
topic_prefix_fc = '/'+fc_name+'/'

#relative information related subscription topics
# relative_pose_topic = ('/vicon/relative_moving_platform_drone/state/pose', PoseStamped)
# relative_twist_topic = ('/vicon/relative_moving_platform_drone/state/twist', TwistStamped)
flightmode_topic = (topic_prefix_fc+'TransmitterInfo',TransmitterInfo)





#define publisher topics
print("drone_name = ",drone_name)
action_topic = ('command/action',ActionMsg)
# yaw_state_topic = ('/'+drone_name+'/cascaded_pid_interface/yaw/state',Float64)


class FlightActivated():
    def __init__(self):
        """Class stores information RL controlled flight is activated."""
        self.flight_activated = False
        #Subscribers
        self.flightmode_subscriber = rospy.Subscriber(flightmode_topic[0],flightmode_topic[1],self.read_flightmode)
        return

    def read_flightmode(self,msg):
        """Reads the status of the ROS controlled parameters and stores the value in the storage class."""
        if msg.ROSControlled == 1:
            self.flight_activated = True
        else:
            self.flight_activated = False
        return


class CascadedPIDInterface():
    def __init__(self):
        #Define class init
        self.flight_activated = FlightActivated()

        #Define variables
        self.action = ActionMsg()
        # self.accel_yaw_rate = Float64MultiArray()
        # self.accel_yaw_rate.data = np.zeros(4)
        # self.relative_pose = PoseStamped()
        # self.relative_twist = TwistStamped()
        self.setpoint_yaw = Float64()
        self.setpoint_yaw.data = 0
                
        #Define publishers
        self.action_publisher = get_publisher(action_topic[0],action_topic[1],queue_size = 0)
        #self.accel_yaw_rate_publisher = get_publisher(accel_yaw_rate_topic[0],accel_yaw_rate_topic[1],queue_size = 0)
        # self.yaw_state_publisher = get_publisher(yaw_state_topic[0],yaw_state_topic[1],queue_size = 0)

        #Define subscribers
        self.x_relative_velocity_control_effort_subscriber = rospy.Subscriber(x_relative_velocity_control_effort_topic[0],x_relative_velocity_control_effort_topic[1],self.read_x_relative_velocity_control_effort)
        self.y_relative_velocity_control_effort_subscriber = rospy.Subscriber(y_relative_velocity_control_effort_topic[0],y_relative_velocity_control_effort_topic[1],self.read_y_relative_velocity_control_effort)
        # self.z_relative_velocity_control_effort_subscriber = rospy.Subscriber(z_relative_velocity_control_effort_topic[0],z_relative_velocity_control_effort_topic[1],self.read_z_relative_velocity_control_effort)
        # self.yaw_control_effort_subscriber = rospy.Subscriber(yaw_control_effort_topic[0],yaw_control_effort_topic[1],self.read_yaw_control_effort)        
        # self.relative_pose_subscriber = rospy.Subscriber(relative_pose_topic[0],relative_pose_topic[1],self.read_relative_pose)
        # self.relative_twist_subscriber = rospy.Subscriber(relative_twist_topic[0],relative_twist_topic[1],self.read_relative_twist)
        self.yaw_setpoint_subscriber = rospy.Subscriber(yaw_setpoint_topic[0],yaw_setpoint_topic[1],self.read_yaw_setpoint)
        self.v_z_subscriber = rospy.Subscriber(v_z_setpoint_topic[0],v_z_setpoint_topic[1],self.read_v_z_setpoint)
        return

    def read_relative_pose(self,msg):
        self.relative_pose = msg
        q = self.relative_pose.pose.orientation
        roll,pitch,yaw = euler_from_quaternion([q.x,q.y,q.z,q.w])
        msg_publish = Float64()
        msg_publish.data = yaw
        # print(getattr(self.observation_relative_state,"rel_yaw"))

        # self.yaw_state_publisher.publish(msg_publish)
        # print("got relative information")
        return
    
    def read_relative_twist(self,msg):
        self.relative_twist = msg
        return
    
    def read_yaw_setpoint(self,msg):
        self.action.yaw = msg.data
        return
    
    def read_v_z_setpoint(self,msg):
        self.action.v_z = -msg.data
        return


    # def read_yaw_control_effort(self,msg):
    #     self.roll_pitch_yawrate_thrust.yaw_rate = msg.data
    #     #self.accel_yaw_rate.data[3] = msg.data
    #     return

    def read_x_relative_velocity_control_effort(self,msg):
        self.action.pitch =np.deg2rad(msg.data) #controll_effort is setpoint for attitude angle, convert from deg2rad
        #self.accel_yaw_rate.data[0] = msg.data

        # print("got control effort x")
        return

    def read_y_relative_velocity_control_effort(self,msg):
        #self.accel_yaw_rate.data[1] = msg.data

        self.action.roll = np.deg2rad(msg.data) #controll_effort is setpoint for attitude angle, convert from deg2rad
        # print("got control effort y")
        return

    # def read_z_relative_velocity_control_effort(self,msg):
    #     #self.accel_yaw_rate.data[2] = msg.data

    #     self.roll_pitch_yawrate_thrust.thrust.z = msg.data
    #     #print("got control effort z")
    #     return




if __name__ == '__main__':

     #Class init
    cascaded_pid_interface = CascadedPIDInterface()
    #Init nodes and subscribers
    
   
    #Init rate
    rate = rospy.Rate(publish_hz)

    #commands to be executed as long as node is up
    while not rospy.is_shutdown():
        if cascaded_pid_interface.flight_activated.flight_activated:
            # cascaded_pid_interface.yaw_setpoint_publisher.publish(cascaded_pid_interface.setpoint_yaw)
            #cascaded_pid_interface.accel_yaw_rate_publisher.publish(cascaded_pid_interface.accel_yaw_rate)
            cascaded_pid_interface.action_publisher.publish(cascaded_pid_interface.action)
            print("ROS control active!")
        else:
            #Publish messages with 0 values but maintaining yaw
            msg = ActionMsg()
            msg.yaw = cascaded_pid_interface.action.yaw
            cascaded_pid_interface.action_publisher.publish(msg)
            print("Sending hover flight attitude (yaw angle maintained)...")

        rate.sleep()