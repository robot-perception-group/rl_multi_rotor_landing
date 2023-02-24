DRONE_NAME=$1
ROS_PORT=$2
GAZ_PORT=$3
ROSIP=$(hostname -I | cut -d' ' -f1) 

# export ROS_IP=$ROSIP;\
export ROS_IP=192.168.7.4
echo "ROS_IP: $ROS_IP"
echo $ROS_MASTER_URI


source ~/src/rl_multi_rotor_landing/rl_multi_rotor_landing_gcs/other_files/setup.bash
roslaunch training_q_learning vicon_filter_platform_data.launch 
