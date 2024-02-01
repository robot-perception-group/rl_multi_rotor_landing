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



#Parameters
rospy.init_node('default') # node name will be overwritten by node name specified in launch file
drone_name = rospy.get_param(rospy.get_name()+'/drone_name','hummingbird')   
publish_hz = float(rospy.get_param(rospy.get_name()+'/publish_hz','10'))

#define subscriber topics
#Controller related subscription topics
x_relative_velocity_control_effort_topic = ('/'+drone_name+'/cascaded_pid_x/il_relative_velocity/control_effort',Float64)
y_relative_velocity_control_effort_topic = ('/'+drone_name+'/cascaded_pid_y/il_relative_velocity/control_effort',Float64)
z_relative_velocity_control_effort_topic = ('/'+drone_name+'/cascaded_pid_z/il_relative_velocity/control_effort',Float64)
yaw_control_effort_topic = ('/'+drone_name+'/cascaded_pid_interface/yaw/control_effort',Float64)

#relative information related subscription topics
relative_pose_topic = ('/'+drone_name+'/landing_simulation/relative_moving_platform_drone/state/pose', PoseStamped)
relative_twist_topic = ('/'+drone_name+'/landing_simulation/relative_moving_platform_drone/state/twist', TwistStamped)




#define publisher topics
print("drone_name = ",drone_name)
roll_pitch_yawrate_thrust_topic = ('command/roll_pitch_yawrate_thrust',RollPitchYawrateThrust)
yaw_setpoint_topic = ('/'+drone_name+'/cascaded_pid_interface/yaw/setpoint',Float64)
yaw_state_topic = ('/'+drone_name+'/cascaded_pid_interface/yaw/state',Float64)


class CascadedPIDInterface():
    def __init__(self):
        #Define variables
        self.roll_pitch_yawrate_thrust = RollPitchYawrateThrust()
        self.accel_yaw_rate = Float64MultiArray()
        self.accel_yaw_rate.data = np.zeros(4)
        self.relative_pose = PoseStamped()
        self.relative_twist = TwistStamped()
        self.setpoint_yaw = Float64()
        self.setpoint_yaw.data = 0
                
        #Define publishers
        self.roll_pitch_yawrate_thrust_publisher = get_publisher(roll_pitch_yawrate_thrust_topic[0],roll_pitch_yawrate_thrust_topic[1],queue_size = 0)
        self.yaw_setpoint_publisher = get_publisher(yaw_setpoint_topic[0],yaw_setpoint_topic[1],queue_size = 0)
        self.yaw_state_publisher = get_publisher(yaw_state_topic[0],yaw_state_topic[1],queue_size = 0)

        #Define subscribers
        self.x_relative_velocity_control_effort_subscriber = rospy.Subscriber(x_relative_velocity_control_effort_topic[0],x_relative_velocity_control_effort_topic[1],self.read_x_relative_velocity_control_effort)
        self.y_relative_velocity_control_effort_subscriber = rospy.Subscriber(y_relative_velocity_control_effort_topic[0],y_relative_velocity_control_effort_topic[1],self.read_y_relative_velocity_control_effort)
        self.z_relative_velocity_control_effort_subscriber = rospy.Subscriber(z_relative_velocity_control_effort_topic[0],z_relative_velocity_control_effort_topic[1],self.read_z_relative_velocity_control_effort)
        self.yaw_control_effort_subscriber = rospy.Subscriber(yaw_control_effort_topic[0],yaw_control_effort_topic[1],self.read_yaw_control_effort)        
        self.relative_pose_subscriber = rospy.Subscriber(relative_pose_topic[0],relative_pose_topic[1],self.read_relative_pose)
        self.relative_twist_subscriber = rospy.Subscriber(relative_twist_topic[0],relative_twist_topic[1],self.read_relative_twist)
        return

    def read_relative_pose(self,msg):
        self.relative_pose = msg
        q = self.relative_pose.pose.orientation
        roll,pitch,yaw = euler_from_quaternion([q.x,q.y,q.z,q.w])
        msg_publish = Float64()
        msg_publish.data = yaw
        # print(getattr(self.observation_relative_state,"rel_yaw"))

        self.yaw_state_publisher.publish(msg_publish)
        # print("got relative information")
        return
    
    def read_relative_twist(self,msg):
        self.relative_twist = msg
        return


    def read_yaw_control_effort(self,msg):
        self.roll_pitch_yawrate_thrust.yaw_rate = msg.data
        #self.accel_yaw_rate.data[3] = msg.data
        return

    def read_x_relative_velocity_control_effort(self,msg):
        self.roll_pitch_yawrate_thrust.pitch =np.deg2rad(msg.data) #controll_effort is setpoint for attitude angle, convert from deg2rad
        #self.accel_yaw_rate.data[0] = msg.data

        # print("got control effort x")
        return

    def read_y_relative_velocity_control_effort(self,msg):
        #self.accel_yaw_rate.data[1] = msg.data

        self.roll_pitch_yawrate_thrust.roll = np.deg2rad(msg.data) #controll_effort is setpoint for attitude angle, convert from deg2rad
        # print("got control effort y")
        return

    def read_z_relative_velocity_control_effort(self,msg):
        #self.accel_yaw_rate.data[2] = msg.data

        self.roll_pitch_yawrate_thrust.thrust.z = msg.data
        #print("got control effort z")
        return




if __name__ == '__main__':

    cascaded_pid_interface = CascadedPIDInterface()
    #Init nodes and subscribers
    
   
    #Init rate
    rate = rospy.Rate(publish_hz)

    #commands to be executed as long as node is up
    while not rospy.is_shutdown():

        cascaded_pid_interface.yaw_setpoint_publisher.publish(cascaded_pid_interface.setpoint_yaw)
        #cascaded_pid_interface.accel_yaw_rate_publisher.publish(cascaded_pid_interface.accel_yaw_rate)
        cascaded_pid_interface.roll_pitch_yawrate_thrust_publisher.publish(cascaded_pid_interface.roll_pitch_yawrate_thrust)
        rate.sleep()