import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import os
import time

#Topics definition
node_name='audio_feedback_node'
feedback_hz = float(rospy.get_param(rospy.get_namespace()+node_name+'/feedback_hz',"30"))
min_altitude = float(rospy.get_param(rospy.get_namespace()+node_name+'/min_altitude',"0.3"))
mp_edge_length = float(rospy.get_param(rospy.get_namespace()+node_name+'/mp_edge_length',"0.5"))

#Parameters definition
drone_pose_topic = ("vicon/drone/pose_enu",PoseStamped)
mp_pose_topic = ("vicon/moving_platform/pose_enu",PoseStamped)

class TrackLandingTrial():
    def __init__(self):
        #Store values
        self.drone_pose = PoseStamped()
        self.mp_pose = PoseStamped()

        #Define Subscribers
        self.drone_pose_subscriber = rospy.Subscriber(drone_pose_topic[0],drone_pose_topic[1],self.read_drone_pose)
        self.mp_pose_subscriber = rospy.Subscriber(mp_pose_topic[0],mp_pose_topic[1],self.read_mp_pose)

        #Variables
        self.start_time = rospy.Time.now()
        self.terminal_criteria_triggered = False
        return

    def read_mp_pose(self,msg):
        self.mp_pose = msg
        return
    
    def read_drone_pose(self,msg):
        self.drone_pose = msg
        return

    def check_terminal_criteria(self):
        drone_pos_x = self.drone_pose.pose.position.x
        drone_pos_y = self.drone_pose.pose.position.y
        drone_pos_z = self.drone_pose.pose.position.z

        mp_pos_x = self.mp_pose.pose.position.x
        mp_pos_y = self.mp_pose.pose.position.y
        
        if drone_pos_z < min_altitude:
            if not self.terminal_criteria_triggered:
                if np.abs(mp_pos_x - drone_pos_x) < mp_edge_length/2 and np.abs(mp_pos_y - drone_pos_y) < mp_edge_length/2:
                    
                    file_path_success = os.path.join(os.environ["ROS_PROJECT_ROOT"],"other_files","sounds","success.mp3")
                    os.system("aplay "+os.environ["ROS_PROJECT_ROOT"]+"/src/LibrePilot/ground/gcs/src/share/sounds/default/whoopsound.wav &") 
                    time.sleep(1)
                    print("Success!")
                    
                else:
                    file_path_failure = os.path.join(os.environ["ROS_PROJECT_ROOT"],"other_files","sounds","failure.mp3") 
                    os.system("aplay "+os.environ["ROS_PROJECT_ROOT"]+"/src/LibrePilot/ground/gcs/src/share/sounds/default/whoopsound.wav &") 
                    print(self.drone_pose.pose.position)
                    print(self.mp_pose.pose.position)
                    print("Failure!")
                self.terminal_criteria_triggered = False
                sound_duration = rospy.Time.now()-self.start_time

                if sound_duration.secs >= 4:
                    self.terminal_criteria_triggered = True

        else:
            self.start_time = rospy.Time.now()
            self.terminal_criteria_triggered = False




if __name__ == '__main__':
    #Init nodes and subscribers
    rospy.init_node(node_name)

    track_landing_trial = TrackLandingTrial()
    rate = rospy.Rate(feedback_hz)

    while not rospy.is_shutdown():
        track_landing_trial.check_terminal_criteria()
        rate.sleep()


