
import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformStamped



node_name = 'velocity_control_frame_publisher_node'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name')

#Odometry topic read by the drone
odom_topic = ('odometry_sensor1/odometry',Odometry)

#Initialize node
rospy.init_node(node_name)
tfBuffer = Buffer()
listener = TransformListener(tfBuffer)
br = TransformBroadcaster()

M_PI = 3.14159265359

def publish_tf_transform_from_odom_frame_to_control_velocity_frame(euler_angle_parent_frame,br,phi, theta, psi,x,y,z):
    """   Function publishes a tf transform from the odom_frame to the stability axes frame."""
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = euler_angle_parent_frame
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


def read_odom_msg(msg): 
    """Function reads the odometry messages of the copter. """
   
    #Get Euler angles from quaternion
    (phi,theta,psi) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    #Publish frame transformation from odometry_sensor1 to velocity_frame
    publish_tf_transform_from_odom_frame_to_control_velocity_frame(msg.header.frame_id,br, phi, theta, psi, msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)
    return

cmd_vel_subscriber = rospy.Subscriber(odom_topic[0],odom_topic[1],read_odom_msg)
rospy.loginfo("publisher node for stability axes frame started ")
rospy.spin()

