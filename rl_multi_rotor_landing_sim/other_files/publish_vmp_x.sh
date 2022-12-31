#Script publishes the velocity of the moving platform on the ROS system specified by ROS port and Gazebo port
vmp=$1
ROS_PORT=$2
GAZ_PORT=$3
ROSIP=$(hostname -I | cut -d' ' -f1) 

export ROS_MASTER_URI=http://$ROSIP:$ROS_PORT;\
export GAZEBO_MASTER_URI=http://$ROSIP:$GAZ_PORT;\
export ROS_IP=$ROSIP;\

echo "ROS_PORT = $ROS_PORT"
echo "GAZEBO_PORT = $GAZ_PORT"
echo "ROS_IP = $ROS_IP"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"

rostopic pub /moving_platform/setpoints/trajectory_speed std_msgs/Float64 "data: $vmp" -r1
