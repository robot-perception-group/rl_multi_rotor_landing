#Script launches the node that computes the relative information between drone and moving platform in a virtual screen
DRONE_NAME=$1
VMAX=$2
RMAX=$3
X_OFFSET=$4
UPDATE_FREQ=$5


# export ROS_MASTER_URI=http://10.42.0.105:11311;\

echo "Set the following parameters for terminal"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"



# #Source workspace
# source $ROS_PROJECT_ROOT/other_files/setup.bash
# source $ROS_PROJECT_ROOT/other_files/activate_training_venv.sh
source $HOME/landing_on_moving_platform/devel/setup.bash


roslaunch training_q_learning vicon_publish_mp_setpoints.launch drone_name:=$DRONE_NAME vmax:=$VMAX rmax:=$RMAX x_offset:=$X_OFFSET update_freq:=$UPDATE_FREQ