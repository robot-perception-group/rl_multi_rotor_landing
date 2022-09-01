#Script launches the node that computes the relative information between drone and moving platform in a virtual screen
ROS_PORT=$1
GAZ_PORT=$2
ROSIP=$(hostname -I | cut -d' ' -f1) 


export ROS_MASTER_URI=http://$ROSIP:$ROS_PORT;\
export GAZEBO_MASTER_URI=http://$ROSIP:$GAZ_PORT;\
export ROS_IP=$ROSIP;\
export ROS_HOSTNAME=$ROSIP;\

echo "Set the following parameters for this terminal:"
echo "ROS_PORT: $ROS_PORT"
echo "GAZ_PORT: $GAZ_PORT"
echo "ROS_IP: $ROS_IP"
echo "ROS_HOSTNAME: $ROS_HOSTNAME"


#Source workspace
source $ROS_PROJECT_ROOT/other_files/setup.bash
source $ROS_PROJECT_ROOT/other_files/activate_training_venv.sh