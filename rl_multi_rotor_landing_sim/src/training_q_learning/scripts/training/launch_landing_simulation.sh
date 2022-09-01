#Source workspace
MAV_NAME=$1
ROS_PORT=$2
GAZ_PORT=$3
ROSIP=$(hostname -I | cut -d' ' -f1) 



export ROS_MASTER_URI=http://$ROSIP:$ROS_PORT;\
export GAZEBO_MASTER_URI=http://$ROSIP:$GAZ_PORT;\
export ROS_IP=$ROSIP;\
export ROS_HOSTNAME=$ROSIP;\

echo "landing_simulation: before sourcing: GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"

source $ROS_PROJECT_ROOT/other_files/setup.bash
source $ROS_PROJECT_ROOT/other_files/activate_training_venv.sh

echo "landing_simulation: after sourcing: GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"




roslaunch training_q_learning landing_simulation.launch mav_name:=$MAV_NAME ros_port:=$ROS_PORT gaz_port:=$GAZ_PORT ros_ip:=$ROS_IP



© 2022 GitHub, Inc.

