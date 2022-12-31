"""This script publishes the copter frame based on the state estimate of the fx as a tf frame."""

import rospy
from geometry_msgs.msg import PoseStamped
from uav_msgs.msg import uav_pose
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformStamped

#Parameters
node_name = 'vicon_state_estimate_copter_frame_publisher_node'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name')

#Topic definition
# pose_topic = ('/vicon/drone/pose',PoseStamped)
pose_topic = ('/'+drone_name+'/pose',PoseStamped)
# pose_fc_topic = ("/fc0/pose",uav_pose)

#Initialize node
rospy.init_node(node_name)
tfBuffer = Buffer()
listener = TransformListener(tfBuffer)
br = TransformBroadcaster()

#Function definitions
def publish_tf_transform_from_world_frame_to_copter_frame(euler_angle_parent_frame,br,pose):
    """   Function publishes a tf transform from the euler_angle_parent_frame frame to the copter axes frame."""
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = euler_angle_parent_frame
    t.child_frame_id = drone_name + "/copter_state_estimate"
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation.x = pose.orientation.x
    t.transform.rotation.y = pose.orientation.y
    t.transform.rotation.z = pose.orientation.z
    t.transform.rotation.w = pose.orientation.w
    br.sendTransform(t)
    print("transform sent...")
    return

def read_pose_msg(msg): 
    """Function reads the pose messages of the copter in the vicon system and calls the function that publishes the tf frame. """
    #Get Euler angles from quaternion
    publish_tf_transform_from_world_frame_to_copter_frame("world",br, msg.pose)
    return

pose_subscriber = rospy.Subscriber(pose_topic[0],pose_topic[1],read_pose_msg)
# pose_subscriber = rospy.Subscriber(pose_fc_topic[0],pose_fc_topic[1],read_uav_pose_msg)
rospy.loginfo("publisher node for copter frame based on state estimate started ")
rospy.spin()

