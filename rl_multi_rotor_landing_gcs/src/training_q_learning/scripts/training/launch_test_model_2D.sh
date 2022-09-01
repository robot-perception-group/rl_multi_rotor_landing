DRONE_NAME=$1
ROS_PORT=$2
GAZ_PORT=$3
ROSIP=$(hostname -I | cut -d' ' -f1) 


export ROS_MASTER_URI=http://$ROSIP:$ROS_PORT;\
export GAZEBO_MASTER_URI=http://$ROSIP:$GAZ_PORT;\
export ROS_IP=$ROSIP;\
export ROS_HOSTNAME=$ROSIP;\


echo "ROS port: $ROS_PORT"
echo "Gazebo port: $GAZ_PORT"
echo "ROS IP: $ROS_IP"
echo "ROS hostname $ROS_HOSTNAME"
echo "launch_training_action_interface.sh: ROS_MASTER_URI=$ROS_MASTER_URI"
echo "launch_training_action_interface.sh: GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"

#Source workspace
source $ROS_PROJECT_ROOT/other_files/setup.bash
source $ROS_PROJECT_ROOT/other_files/activate_training_venv.sh

roslaunch training_q_learning test_model_2D.launch drone_name:=$DRONE_NAME ros_port:=$ROS_PORT gaz_port:=$GAZ_PORT ros_ip:=$ROS_IP