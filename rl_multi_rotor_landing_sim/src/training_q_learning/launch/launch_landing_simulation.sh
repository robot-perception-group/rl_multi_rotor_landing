#Source workspace
MAV_NAME=$1
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

#Source workspace
source $ROS_PROJECT_ROOT/other_files/setup.bash
source $ROS_PROJECT_ROOT/other_files/activate_training_venv.sh

roslaunch training_q_learning landing_simulation.launch mav_name:=$MAV_NAME