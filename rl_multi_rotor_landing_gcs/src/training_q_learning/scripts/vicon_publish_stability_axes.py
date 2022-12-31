
import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from uav_msgs.msg import uav_pose
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformStamped



node_name = 'vicon_stability_frame_publisher_node'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name')

#Topic defiition    
# pose_topic = ('/vicon/drone/pose',PoseStamped)
pose_topic = ('/'+drone_name+'/pose',PoseStamped)
# pose_fc_topic = ("/fc0/pose",uav_pose)

#Initialize node
rospy.init_node(node_name)
tfBuffer = Buffer()
listener = TransformListener(tfBuffer)
br = TransformBroadcaster()


def publish_stability_axes_as_tf_frame(parent_frame,br,phi,theta,psi,x,y,z):
    """Function publishes the stability axes frame w.r.t. the world frame."""
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame
    t.child_frame_id = drone_name + "/stability_axes"
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    q = quaternion_from_euler(0, 0, psi)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]    
    br.sendTransform(t)
    return

def read_pose_msg(msg): 
    """Function reads the pose messages of the copter in the vicon system. """
   
    #Get Euler angles from quaternion
    (phi,theta,psi) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

    #Publish frame transformation 
    publish_stability_axes_as_tf_frame(msg.header.frame_id,br, phi, theta, psi, msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)
    return

pose_subscriber = rospy.Subscriber(pose_topic[0],pose_topic[1],read_pose_msg)
# pose_subscriber = rospy.Subscriber(pose_fc_topic[0],pose_fc_topic[1],read_uav_pose_msg)
rospy.loginfo("publisher node for stability axes frame started ")
rospy.spin()

