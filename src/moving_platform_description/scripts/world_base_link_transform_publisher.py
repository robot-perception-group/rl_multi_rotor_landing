#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_ros
from gazebo_msgs.msg import ModelStates

#Define subscriber topics
gazebo_model_state_topic = ('/gazebo/model_states',ModelStates)


#This script reads the gazebo model state and publishes a tf frame called baselink at the same position as the current gazebo model position is


frequency = 100



class PositionOrientationState():
    #Define and initialize the class. These values will be overridden with received data
    def __init__(self):
        """Class for storing the current position and orientation of the moving platform"""
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'world'
        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0
        self.pose.pose.orientation.x = 0
        self.pose.pose.orientation.y = 0
        self.pose.pose.orientation.z = 0
        self.pose.pose.orientation.w = 0




moving_platform_state = PositionOrientationState()

#######################
#IMPORTANT: The pose is given in world frame. Nevertheless, for consitency reasons with the urdf of the moving platform the transformation
#has to be published between the world_link frame and the moving_platform/base_link frame
#######################

def read_model_state(msg):
    """Read model state of moving platform."""

    #Retrieve moving platform information from moving platform and save it in class
    object_coordinates_pose_list = getattr(msg,"pose")
    model_name_list = getattr(msg,"name")
    idx_mp = model_name_list.index("moving_platform")
    moving_platform_state.pose.header.frame_id = 'world'
    moving_platform_state.pose.pose = object_coordinates_pose_list[idx_mp]

    publish_world_to_world_link_transform_message()
    publish_transform_message()
    return


def publish_world_to_world_link_transform_message():
    """Function publishes the required tf transform from world frame to an additional frame called world_link in which the platform movement is described."""
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "world_link"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1    
    br.sendTransform(t)

def publish_transform_message():
    """Publish the transformation message to describe the motion of the moving platform w.r.t. to the world_link frame."""
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world_link"
    t.child_frame_id = "moving_platform/base_link"
    t.transform.translation.x = moving_platform_state.pose.pose.position.x
    t.transform.translation.y = moving_platform_state.pose.pose.position.y
    t.transform.translation.z = moving_platform_state.pose.pose.position.z
    t.transform.rotation.x = moving_platform_state.pose.pose.orientation.x
    t.transform.rotation.y = moving_platform_state.pose.pose.orientation.y
    t.transform.rotation.z = moving_platform_state.pose.pose.orientation.z
    t.transform.rotation.w = moving_platform_state.pose.pose.orientation.w    
    br.sendTransform(t) 
    return





if __name__ == '__main__':
    rospy.init_node('moving_platform_world_base_link_transform_publisher_node')
    rospy.loginfo("Startup")
    channelssubscriber = rospy.Subscriber(gazebo_model_state_topic[0],gazebo_model_state_topic[1],read_model_state)
    rospy.loginfo("pose_subscriber started")
    rate = rospy.Rate(frequency)
    
    while not rospy.is_shutdown():
        rate.sleep()










