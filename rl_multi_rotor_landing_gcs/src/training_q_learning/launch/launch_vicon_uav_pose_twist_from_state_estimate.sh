DRONE_NAME=$1
ROS_PORT=$2
GAZ_PORT=$3
ROSIP=$(hostname -I | cut -d' ' -f1) 

export ROS_IP=$ROSIP;\
echo "ROS_IP: $ROS_IP"

source $HOME/rl_multi_rotor_landing/devel/setup.bash

roslaunch training_q_learning vicon_uav_pose_twist_from_state_estimate.launch drone_name:=$DRONE_NAME