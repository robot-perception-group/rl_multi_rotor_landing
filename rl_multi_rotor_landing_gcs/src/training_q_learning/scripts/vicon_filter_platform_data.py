"""This scripts reads vicon data and redistributes it to required topics to run the landing algorithm. It performs a transformation from ENU to NED frame."""


from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from copy import deepcopy
import rospy
from training_q_learning.msg import LandingSimulationObjectState
import math
import numpy as np


#Get parameters
node_name='filter_platform_data'
vicon_interface_publish_hz = float(rospy.get_param(rospy.get_namespace()+node_name+'/vicon_interface_publish_hz',"10"))
vicon_drone_id = rospy.get_param(rospy.get_namespace()+node_name+'/vicon_drone_id','drone')
vicon_mp_id = rospy.get_param(rospy.get_namespace()+node_name+'/vicon_mp_id','moving_platform')
fc_name = rospy.get_param(rospy.get_namespace()+node_name+'/fc_name','fc0')

#Define topics
#vicon subscriber topics

vicon_mp_pos_topic = ('/vicon/moving_platform/pose_enu',PoseStamped)
vicon_mp_vel_topic = ('/vicon/moving_platform/twist_enu',TwistStamped)

#vicon publisher topics
vicon_mp_pos_filtered_topic = ('/vicon/moving_platform/pose_enu_filtered',PoseStamped)
vicon_mp_vel_filtered_topic = ('/vicon/moving_platform/twist_enu_filtered',TwistStamped)




min_cutoff_pos_x = 1
min_cutoff_pos_y = 1
min_cutoff_pos_z = 1
min_cutoff_vel_x = 0.3
min_cutoff_vel_y = 0.3
min_cutoff_vel_z = 0.3
beta_pos_x = 0 
beta_pos_y = 0 
beta_pos_z = 0
beta_vel_x = 0
beta_vel_y = 0
beta_vel_z = 0
d_cutoff_pos_x = 1
d_cutoff_pos_y = 1
d_cutoff_pos_z = 1
d_cutoff_vel_x = 1
d_cutoff_vel_y = 1
d_cutoff_vel_z = 1
dx0_pos_x = 0
dx0_pos_y = 0
dx0_pos_z = 0
dx0_vel_x = 0
dx0_vel_y = 0
dx0_vel_z = 0



class OneEuroFilter:
    def __init__(self, t0, x0, dx0=0, min_cutoff=1, beta=0,d_cutoff=1):
        """Initialize the one euro filter. Based on https://jaantollander.com/post/noise-filtering-using-one-euro-filter/"""
        # The parameters.
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        # Previous values.
        self.x_prev = float(x0)
        self.dx_prev = float(dx0)
        self.t_prev = float(t0)
        return

    def smoothing_factor(self,t_e, cutoff):
        r = 2 * math.pi * cutoff * t_e
        return r / (r + 1)

    def exponential_smoothing(self,a, x, x_prev):
        return a * x + (1 - a) * x_prev

    def filter(self, t, x):
        """Compute the filtered signal."""
        t_e = t - self.t_prev

        # The filtered derivative of the signal.
        a_d = self.smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = self.exponential_smoothing(a_d, dx, self.dx_prev)

        # The filtered signal.
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self.smoothing_factor(t_e, cutoff)
        x_hat = self.exponential_smoothing(a, x, self.x_prev)

        # Memorize the previous values.
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t

        return x_hat




#Class definition
class SmoothPlatformData():
    def __init__(self):
        """Function stores the pose and twist data of the platform received via the vicon system. It applies simple smoothing techniques to reduce noise induced
        through vibrations caused by the movement of the platform on uneven ground."""
        #Data storage

        #Define subscribers
        self.mp_pos_subscriber = rospy.Subscriber(vicon_mp_pos_topic[0],vicon_mp_pos_topic[1],self.read_mp_pose)
        # self.mp_vel_subscriber = rospy.Subscriber(vicon_mp_vel_topic[0],vicon_mp_vel_topic[1],self.read_mp_twist)

        #Define publishers
        self.mp_pos_filtered_publisher = rospy.Publisher(vicon_mp_pos_filtered_topic[0],vicon_mp_pos_filtered_topic[1],queue_size=0)
        self.mp_vel_filtered_publisher = rospy.Publisher(vicon_mp_vel_filtered_topic[0],vicon_mp_vel_filtered_topic[1],queue_size=0)

        #Store variables
        self.mp_pos_filtered = PoseStamped()
        self.mp_vel_filtered = TwistStamped()

        #Define one euro filters
        self.pos_x_one_euro_filter = OneEuroFilter(0,0,dx0=dx0_pos_x, min_cutoff=min_cutoff_pos_x, beta=beta_pos_x,d_cutoff=d_cutoff_pos_x)
        self.pos_y_one_euro_filter = OneEuroFilter(0,0,dx0=dx0_pos_y, min_cutoff=min_cutoff_pos_y, beta=beta_pos_y,d_cutoff=d_cutoff_pos_y)
        self.pos_z_one_euro_filter = OneEuroFilter(0,0,dx0=dx0_pos_z, min_cutoff=min_cutoff_pos_z, beta=beta_pos_z,d_cutoff=d_cutoff_pos_z)

        #Define cut off freq for low pass filter used for filtering the velocity
        self.cut_off_freq = 7 #hz


        # self.vel_x_one_euro_filter = OneEuroFilter(0,0,dx0=dx0_vel_x, min_cutoff=min_cutoff_vel_x, beta=beta_vel_x,d_cutoff=d_cutoff_vel_x)
        # self.vel_y_one_euro_filter = OneEuroFilter(0,0,dx0=dx0_vel_y, min_cutoff=min_cutoff_vel_y, beta=beta_vel_y,d_cutoff=d_cutoff_vel_y)
        # self.vel_z_one_euro_filter = OneEuroFilter(0,0,dx0=dx0_vel_z, min_cutoff=min_cutoff_vel_z, beta=beta_vel_z,d_cutoff=d_cutoff_vel_z)
        return
    

    def read_mp_pose(self,msg):
        #extract time for filters
        t = float(msg.header.stamp.to_sec())

        #extract values - no filter applied
        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y
        pos_z = msg.pose.position.z
        pos_x_filtered = pos_x
        pos_y_filtered = pos_y
        pos_z_filtered = pos_z

        #Apply one euro filter
        # pos_x_filtered = self.pos_x_one_euro_filter.filter(t,pos_x)
        # pos_y_filtered = self.pos_y_one_euro_filter.filter(t,pos_y)
        # pos_z_filtered = self.pos_z_one_euro_filter.filter(t,pos_z)



        #Publish filtered values
        self.publish_filtered_mp_pos()
        self.publish_filtered_vel_from_pos(msg)
                
        #Store filtered variables
        self.mp_pos_filtered.pose.position.x = pos_x_filtered
        self.mp_pos_filtered.pose.position.y = pos_y_filtered
        self.mp_pos_filtered.pose.position.z = pos_z_filtered
        return

    def publish_filtered_vel_from_pos(self,msg):
        #Use discrete first order butterworth filter to reduce noise
        cut_off_ang_freq = self.cut_off_freq

        #Determine time between two velocity messages
        delta_t_sample = msg.header.stamp.to_sec()-self.mp_vel_filtered.header.stamp.to_sec()

        #Compute the relative acceleration based on current and previous pose message using first order Butterworth low pass filter
        if delta_t_sample:
            v_x = np.exp(-cut_off_ang_freq*delta_t_sample)*self.mp_vel_filtered.twist.linear.x + (1-np.exp(-cut_off_ang_freq*delta_t_sample))*(msg.pose.position.x - self.mp_pos_filtered.pose.position.x) / delta_t_sample
            v_y = np.exp(-cut_off_ang_freq*delta_t_sample)*self.mp_vel_filtered.twist.linear.y + (1-np.exp(-cut_off_ang_freq*delta_t_sample))*(msg.pose.position.y - self.mp_pos_filtered.pose.position.y) / delta_t_sample
            v_z = np.exp(-cut_off_ang_freq*delta_t_sample)*self.mp_vel_filtered.twist.linear.z + (1-np.exp(-cut_off_ang_freq*delta_t_sample))*(msg.pose.position.z - self.mp_pos_filtered.pose.position.z) / delta_t_sample
            self.mp_vel_filtered.twist.linear.x = v_x
            self.mp_vel_filtered.twist.linear.y = v_y
            self.mp_vel_filtered.twist.linear.z = v_z
            self.mp_vel_filtered.header.stamp = msg.header.stamp
        self.publish_filtered_mp_vel()
        return


    def read_mp_twist(self,msg):
        t = float(msg.header.stamp.to_sec())

        #extract values
        vel_x = msg.twist.linear.x
        vel_y = msg.twist.linear.y
        vel_z = msg.twist.linear.z

        #Apply filter
        vel_x_filtered = self.vel_x_one_euro_filter.filter(t,vel_x)
        vel_y_filtered = self.vel_y_one_euro_filter.filter(t,vel_y)
        vel_z_filtered = self.vel_z_one_euro_filter.filter(t,vel_z)

        #Store filtered variables
        self.mp_vel_filtered.twist.linear.x = vel_x_filtered
        self.mp_vel_filtered.twist.linear.y = vel_y_filtered
        self.mp_vel_filtered.twist.linear.z = vel_z_filtered

        #Publish filtered values
        self.publish_filtered_mp_vel()
        return

    def publish_filtered_mp_pos(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose = self.mp_pos_filtered.pose
        self.mp_pos_filtered_publisher.publish(msg)
        return

    def publish_filtered_mp_vel(self):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.twist = self.mp_vel_filtered.twist   
        self.mp_vel_filtered_publisher.publish(msg)
     
        return



if __name__ == '__main__':
    #Init nodes 
    rospy.init_node(node_name)
    smooth_platform_data = SmoothPlatformData() 
    rospy.loginfo("filtering of platform data started...")
    rospy.spin()
