DRONE_NAME=$1
ROS_PORT=$2
GAZ_PORT=$3
ROSIP=$(hostname -I | cut -d' ' -f1) 

export ROS_IP=$ROSIP;\
echo "ROS_IP: $ROS_IP"

roslaunch vrpn_client_ros landing_on_moving_platform.launch server:=XXX.XXX.XX.X # Enter the IP address of your Vicon base computer here
