DRONE_NAME=$1
ROS_PORT=$2
GAZ_PORT=$3
ROSIP=$(hostname -I | cut -d' ' -f1) 

# export ROS_IP=$ROSIP;\
export ROS_IP=192.168.7.4
echo "ROS_IP: $ROS_IP"
echo $ROS_MASTER_URI

roslaunch vrpn_client_ros landing_on_moving_platform.launch server:=192.168.10.1 # Enter the IP address of your Vicon base computer here
