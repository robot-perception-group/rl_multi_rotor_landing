'''
Function publishes the stability axes as tf frame. The stability axes xy-plane is always parallel to earth surface but the x-axes is aligned with the x-axis of the body-fixed
frame.
For this purpose, the odometry topic of the drone that is published by the rotorS package is read and whenever a message is received a the stability axes is published
as tf frame.
'''

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformStamped

node_name = 'stability_frame_publisher_node'
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name')

#Odometry topic of the drone
odom_topic = ('odometry_sensor1/odometry',Odometry)

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

def read_odom_msg(msg): 
    """Function reads the odometry messages of the drone and triggers publishing of stability frame. For this purpose it reads the current translational and orientational position
        of the odometry frame w.r.t. to the world frame and uses this information to publish the stability frame. The stability frame's xy-plane is parallel to the world's
        xy-plane and the x-axis is vertically aligned with the x-axis of the body-fixed frame of the drone.
    """
    #Get Euler angles from quaternion
    (phi,theta,psi) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    #Publish frame transformation from odometry_sensor1 to stability frame
    publish_stability_axes_as_tf_frame(msg.header.frame_id,br, phi, theta, psi, msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)
    return

odom_subscriber = rospy.Subscriber(odom_topic[0],odom_topic[1],read_odom_msg)
rospy.loginfo("publisher node for stability axes frame started ")
rospy.spin()

