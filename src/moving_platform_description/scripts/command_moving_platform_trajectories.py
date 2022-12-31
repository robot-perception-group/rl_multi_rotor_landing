#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import tf
import numpy as np
from std_msgs.msg import Float64, Bool 
from training_q_learning.parameters import Parameters
from training_q_learning.srv import ResetRandomSeed, ResetRandomSeedResponse

#Define subscriber topics
pose_publisher_topic = ('moving_platform/commanded/pose',Pose)
gazebo_pose_publisher_topic = ('/gazebo/set_model_state',ModelState)
gazebo_model_state_publish_service_topic = ('/gazebo/set_model_state',SetModelState) 

drone_name = rospy.get_param("command_moving_platform_trajectories_node/drone_name","iris")
reset_simulation_topic = ("/"+drone_name+"/training/reset_simulation",Bool)


trajectory_speed_topic = ('moving_platform/setpoints/trajectory_speed',Float64)
trajectory_speed_lateral_topic = ('moving_platform/setpoints/trajectory_speed_lateral',Float64)
trajectory_radius_topic = ('moving_platform/setpoints/trajectory_radius',Float64)
trajectory_radius_lateral_topic = ('moving_platform/setpoints/trajectory_radius_lateral',Float64)


class TrajectoryGenerator:
    """Class that creates a trajectory and publishes it using geometry_msgs/Pose messages"""

    def __init__(self):
        """Class contains data required in order to update the current position of the moving platform."""
        #Access parameters
        self.parameters = Parameters()
        
        #Get parameters
        self.trajectory_type = rospy.get_param("command_moving_platform_trajectories_node/trajectory_type","straight")
        self.trajectory_speed = float(rospy.get_param("command_moving_platform_trajectories_node/trajectory_speed","1")) #[m/s]
        self.trajectory_frequency = float(rospy.get_param("command_moving_platform_trajectories_node/trajectory_frequency","100")) #[hz]
        self.trajectory_scale_factor = float(rospy.get_param("command_moving_platform_trajectories_node/trajectory_scale_factor","1")) #[-]
        self.trajectory_start_position = rospy.get_param("command_moving_platform_trajectories_node/trajectory_start_position",{'x':0,'y':0,'z':0}) #[m]
        self.trajectory_start_orientation = rospy.get_param("command_moving_platform_trajectories_node/trajectory_start_orientation",{'phi':0,'theta':0,'psi':0}) #{rad}
        self.trajectory_radius = float(rospy.get_param("command_moving_platform_trajectories_node/trajectory_radius","10")) #[m]
        self.trajectory_radius_vertical = float(rospy.get_param("command_moving_platform_trajectories_node/trajectory_radius_vertical","10")) #[m]
        self.trajectory_speed_vertical = float(rospy.get_param("command_moving_platform_trajectories_node/trajectory_speed_vertical","1")) #[m/s]
        self.trajectory_type_vertical = rospy.get_param("command_moving_platform_trajectories_node/trajectory_type_vertical","straight")
        self.trajectory_radius_lateral = float(rospy.get_param("command_moving_platform_trajectories_node/trajectory_radius_lateral","10")) #[m]
        self.trajectory_speed_lateral = float(rospy.get_param("command_moving_platform_trajectories_node/trajectory_speed_lateral","1")) #[m/s]

        #Convert dict values to float
        for k, v in self.trajectory_start_position.items():
            self.trajectory_start_position[k] = float(v)

        for k, v in self.trajectory_start_orientation.items():
            self.trajectory_start_orientation[k] = float(v)

        #Current pose values
        #position
        self.x = self.trajectory_start_position['x']
        self.y = self.trajectory_start_position['y'] 
        self.z = self.trajectory_start_position['z']
        self.u = 0
        self.v = 0
        self.w = 0
        #orientation in Euler angles
        self.phi  = self.trajectory_start_orientation['phi']
        self.theta = self.trajectory_start_orientation['theta']
        self.psi = self.trajectory_start_orientation['psi']

        self.t = 0
        self.delta_t = 1 / self.trajectory_frequency #sec        

        self.pose_publisher = rospy.Publisher(pose_publisher_topic[0],pose_publisher_topic[1],queue_size = 3)
        self.gazebo_pose_publisher = rospy.Publisher(gazebo_pose_publisher_topic[0],gazebo_pose_publisher_topic[1],queue_size = 3)
        self.set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        return

    def read_trajectory_speed(self,msg):
        """Function reads the commanded trajectory speed for longitudinal motion from the associated ROS topic."""
        self.trajectory_speed = msg.data
        return

    def read_trajectory_radius(self,msg):
        """Function reads the commanded trajectory radius for longitudinal motion from the associated ROS topic."""
        self.trajectory_radius = msg.data
        return

    def read_trajectory_speed_lateral(self,msg):
        """Function reads the commanded trajectory speed for the lateral motion from the associated ROS topic."""
        self.trajectory_speed_lateral = msg.data
        return

    def read_trajectory_radius_lateral(self,msg):
        """Function reads the commanded trajectory radius for the lateral motion from the associated ROS topic."""
        self.trajectory_radius_lateral = msg.data
        return

    def compute_trajectory_circle(self):
        """Function computes a circular trajectory for the moving platform."""
        omega = self.trajectory_speed / self.trajectory_radius
        x = self.trajectory_radius*np.cos(omega*self.t) + self.trajectory_start_position["x"]
        y = self.trajectory_radius*np.sin(omega*self.t)+ self.trajectory_start_position["y"]
        u = self.trajectory_radius*omega*(-np.sin(omega*self.t))
        v = self.trajectory_radius*omega*(np.cos(omega*self.t))
        return x,y,u,v

    def compute_trajectory_straight(self):
        """Function computes a straight trajectory for the moving platform."""
        x = self.x + self.trajectory_speed * self.delta_t
        y = self.y
        u = self.trajectory_speed
        v = self.v
        return x,y,u,v

    def compute_trajectory_rectiliar_periodic_straight(self):
        """Function computes a rectiliar periodic trajectory for the moving platform."""
        omega = self.trajectory_speed / (self.trajectory_radius)
        omega_lateral = self.trajectory_speed_lateral / self.trajectory_radius_lateral
        x = self.trajectory_radius*np.sin(omega*self.t) + self.trajectory_start_position["x"]
        y = self.trajectory_radius_lateral*np.sin(omega_lateral*self.t) + self.trajectory_start_position["y"]
        u = self.trajectory_radius*omega*np.cos(omega*self.t)
        v = self.trajectory_radius_lateral*omega_lateral*np.cos(omega_lateral*self.t)
        return x,y,u,v
    
    def compute_vertical_trajectory_straight(self):
        """Function computes a vertical, straight trajectory for the moving platform"""
        z = self.z + self.trajectory_speed_vertical * self.delta_t
        w = self.trajectory_speed_vertical
        return z,w

    def compute_vertical_trajectory_rectiliar_periodic_straight(self):
        """Function computes a rectiliar periodic trajectory."""
        omega_vertical = self.trajectory_speed_vertical / self.trajectory_radius_vertical
        z = self.trajectory_radius_vertical*np.sin(omega_vertical*self.t) + self.trajectory_start_position["z"]
        w = self.trajectory_radius_vertical*omega_vertical*np.cos(omega_vertical*self.t)        
        return z,w

    def compute_trajectory(self):
        """Function computes the trajectory based on arguments trajectory_type, trajectory_velocity, trajectory_frequency """
        if self.trajectory_type in ["circle","straight","rectilinear_periodic_straight"] and self.trajectory_type_vertical in ["rectilinear_periodic_straight","straight"]:
            #Define horizontal movement
            if self.trajectory_type == "circle":
                (x,y,u,v) = self.compute_trajectory_circle()
                
            if self.trajectory_type == "straight":
                (x,y,u,v) = self.compute_trajectory_straight()

            if self.trajectory_type == "rectilinear_periodic_straight":
                (x,y,u,v) = self.compute_trajectory_rectiliar_periodic_straight()

            if self.trajectory_type_vertical == "rectilinear_periodic_straight":
                (z,w) = self.compute_vertical_trajectory_rectiliar_periodic_straight()
            
            if self.trajectory_type_vertical == "straight":
                (z,w) = self.compute_vertical_trajectory_straight()

            self.x = x
            self.y = y
            self.z = z  
            self.u = u
            self.v = v   
            self.w = w    
            self.t = self.t + self.delta_t  
        else:
            raise ValueError
            print("Specified horizontal or vertical trajectory type not known! Aborting...")
        return


    def publish_trajectory(self):
        """Function publishes the updated platform position to the ROS network."""
        msg = Pose()
        msg.position.x = self.x
        msg.position.y = self.y
        msg.position.z = self.z
        quat = tf.transformations.quaternion_from_euler(self.phi,self.theta,self.psi)
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        
      
        self.pose_publisher.publish(msg)

        msg_gazebo = ModelState()
        msg_gazebo.model_name = 'moving_platform'
        msg_gazebo.reference_frame = 'ground_plane'
        msg_gazebo.pose.position.x = self.x
        msg_gazebo.pose.position.y = self.y
        msg_gazebo.pose.position.z = self.z
        quat = tf.transformations.quaternion_from_euler(self.phi,self.theta,self.psi)
        msg_gazebo.pose.orientation.x = quat[0]
        msg_gazebo.pose.orientation.y = quat[1]
        msg_gazebo.pose.orientation.z = quat[2]
        msg_gazebo.pose.orientation.w = quat[3]
        msg_gazebo.twist.linear.x = self.u
        msg_gazebo.twist.linear.y = self.v
        msg_gazebo.twist.linear.z = self.w

        self.gazebo_pose_publisher.publish(msg_gazebo)
        self.set_state_service(msg_gazebo)
        return
    
    def read_reset(self,msg):
        """Function resets time to a random value within an interval of episode_length whenever the message value is true."""
        if msg.data:
            self.t = np.random.uniform(0,self.parameters.rl_parameters.max_num_timesteps_episode*self.parameters.rl_parameters.running_step_time)
        return
    
    def reset_random_seed(self,req):
        """Function handles the service request to reset the seed for the random number generator."""
        seed = None if req.seed == 'None' else int(req.seed)
        print("Set seed for random initial values to",seed)
        np.random.seed(seed)
        return ResetRandomSeedResponse()


if __name__ == '__main__':
    rospy.init_node('trajectory_generator_node', anonymous=True)
    trajectory_generator = TrajectoryGenerator()
    s = rospy.Service('/moving_platform/reset_random_seed', ResetRandomSeed, trajectory_generator.reset_random_seed)
    trajectory_speed_subscriber = rospy.Subscriber(trajectory_speed_topic[0],trajectory_speed_topic[1],trajectory_generator.read_trajectory_speed)
    trajectory_radius_subscriber = rospy.Subscriber(trajectory_radius_topic[0],trajectory_radius_topic[1],trajectory_generator.read_trajectory_radius)
    trajectory_speed_lateral_subscriber = rospy.Subscriber(trajectory_speed_lateral_topic[0],trajectory_speed_lateral_topic[1],trajectory_generator.read_trajectory_speed_lateral)
    trajectory_radius_lateral_subscriber = rospy.Subscriber(trajectory_radius_lateral_topic[0],trajectory_radius_lateral_topic[1],trajectory_generator.read_trajectory_radius_lateral)
    reset_simulation_subscriber = rospy.Subscriber(reset_simulation_topic[0],reset_simulation_topic[1],trajectory_generator.read_reset)
    rate = rospy.Rate(trajectory_generator.trajectory_frequency)
    rospy.wait_for_service('/gazebo/set_model_state')
    while not rospy.is_shutdown():
        trajectory_generator.compute_trajectory()
        trajectory_generator.publish_trajectory()
        rate.sleep()
