#Script launches the node that computes the relative information between drone and moving platform in a virtual screen
DRONE_NAME=$1
ROS_PORT=$2
GAZ_PORT=$3
ROSIP=$(hostname -I | cut -d' ' -f1) 
ROS_PORT=11311


# export ROS_MASTER_URI=http://10.42.0.105:11311;\
# export GAZEBO_MASTER_URI=http://$ROSIP:$GAZ_PORT;\
export ROS_IP=$ROSIP;\
# export ROS_HOSTNAME=$ROSIP;\

echo "Set the following parameters for terminal"
echo "ROS_PORT: $ROS_PORT"
echo "GAZ_PORT: $GAZ_PORT"
echo "ROS_IP: $ROS_IP"
echo "ROS_HOSTNAME: $ROS_HOSTMAME"


# #Source workspace
# source $ROS_PROJECT_ROOT/other_files/setup.bash
# source $ROS_PROJECT_ROOT/other_files/activate_training_venv.sh
source $HOME/landing_on_moving_platform/devel/setup.bash


roslaunch training_q_learning vicon_publish_stability_axes.launch drone_name:=$DRONE_NAME ros_port:=$ROS_PORT gaz_port:=$GAZ_PORT ros_ip:=$ROS_IP