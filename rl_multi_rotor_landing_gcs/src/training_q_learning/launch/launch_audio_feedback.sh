min_altitude=$1
mp_edge_length=$2
export ROS_IP=192.168.7.4
echo "ROS_IP: $ROS_IP"
echo $ROS_MASTER_URI

source ~/src/rl_multi_rotor_landing/rl_multi_rotor_landing_gcs/other_files/setup.bash
roslaunch training_q_learning audio_feedback.launch  min_altitude:=$min_altitude mp_edge_length:=$mp_edge_length