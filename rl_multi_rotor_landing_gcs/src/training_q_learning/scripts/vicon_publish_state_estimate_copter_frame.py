import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from uav_msgs.msg import uav_pose
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformStamped



node_name = 'vicon_state_estimate_copter_frame_publisher_node'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name')

#Odometry topic read by the drone
# pose_topic = ('/vicon/drone/pose',PoseStamped)
pose_topic = ('/'+drone_name+'/pose',PoseStamped)
# pose_fc_topic = ("/fc0/pose",uav_pose)

#Initialize node
rospy.init_node(node_name)
tfBuffer = Buffer()
listener = TransformListener(tfBuffer)
br = TransformBroadcaster()

M_PI = 3.14159265359

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

# def publish_tf_transform_from_ned_pose_frame_to_stability_frame(euler_angle_parent_frame,br,phi, theta, psi,x,y,z):
#     """   Function publishes a tf transform from the odom_frame to the stability axes frame."""
#     t = TransformStamped()
#     t.header.stamp = rospy.Time.now()
#     t.header.frame_id = euler_angle_parent_frame
#     t.child_frame_id = drone_name + "/stability_axes"
#     t.transform.translation.x = y
#     t.transform.translation.y = x
#     t.transform.translation.z = -z
#     q = quaternion_from_euler(0, 0, -psi)
#     t.transform.rotation.x = q[0]
#     t.transform.rotation.y = q[1]
#     t.transform.rotation.z = q[2]
#     t.transform.rotation.w = q[3]    
#     br.sendTransform(t)
#     return

# def read_uav_pose_msg(msg): 
#     """Function reads the pose messages of the copter in the vicon system. """
   
#     #Get Euler angles from quaternion
#     (phi,theta,psi) = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

#     #Publish frame transformation from odometry_sensor1 to velocity_frame
#     publish_tf_transform_from_pose_frame_to_stability_frame(msg.header.frame_id,br, phi, theta, psi, msg.position.x,msg.position.y,msg.position.z)
#     # publish_tf_transform_from_ned_pose_frame_to_stability_frame(msg.header.frame_id,br, phi, theta, psi, msg.position.x,msg.position.y,msg.position.z)
#     return

def read_pose_msg(msg): 
    """Function reads the pose messages of the copter in the vicon system. """
    #Get Euler angles from quaternion
    (phi,theta,psi) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    # print((180/3.14)*phi,(180/3.14)*theta,(180/3.14)*psi)

    #Publish frame transformation from odometry_sensor1 to velocity_frame
    publish_tf_transform_from_world_frame_to_copter_frame("world",br, msg.pose)
    return

pose_subscriber = rospy.Subscriber(pose_topic[0],pose_topic[1],read_pose_msg)
# pose_subscriber = rospy.Subscriber(pose_fc_topic[0],pose_fc_topic[1],read_uav_pose_msg)
rospy.loginfo("publisher node for copter frame based on state estimate started ")
rospy.spin()

