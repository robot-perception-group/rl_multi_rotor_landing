#Script launches the node that computes the relative information between drone and moving platform in a virtual screen
DRONE_NAME=$1
ROS_PORT=$2
GAZ_PORT=$3
ROSIP=$(hostname -I | cut -d' ' -f1) 


export ROS_MASTER_URI=http://$ROSIP:$ROS_PORT;\
export GAZEBO_MASTER_URI=http://$ROSIP:$GAZ_PORT;\
export ROS_IP=$ROSIP;\
export ROS_HOSTNAME=$ROSIP;\


echo $ROS_PORT
echo $GAZ_PORT
echo $ROS_IP
echo $ROS_HOSTNAME

#Source workspace
source $ROS_PROJECT_ROOT/other_files/setup.bash
source $ROS_PROJECT_ROOT/other_files/activate_training_venv.sh

roslaunch training_q_learning hitl_interface.launch drone_name:=$DRONE_NAME ros_port:=$ROS_PORT gaz_port:=$GAZ_PORT ros_ip:=$ROS_IP