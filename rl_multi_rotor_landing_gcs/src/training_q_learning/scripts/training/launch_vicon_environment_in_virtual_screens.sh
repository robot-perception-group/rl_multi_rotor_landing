#Script launches landing simulation in virtual screens
SIM_ID=$1
MAV_NAME=$2
ROS_PORT=$3 
GAZ_PORT=$4
ROSIP=$(hostname -I | cut -d' ' -f1) 


# export ROS_MASTER_URI=http://$ROSIP:$ROS_PORT;\
# export GAZEBO_MASTER_URI=http://$ROSIP:$GAZ_PORT;\
# export ROS_IP=$ROSIP;\
# export ROS_HOSTNAME=$ROSIP;\

# echo "SIM_ID=$SIM_ID"
# echo "ROS_PORT=$ROS_PORT"
# echo "GAZ_PORT=$GAZ_PORT"
# echo "ROS_IP=$ROS_IP"
# echo "ROS_HOSTNAME=$ROS_HOSTNAME"

echo "Launching vicon bridge..."
screen -d -m -S "$(echo $SIM_ID)_bridge" bash -i -c "$(echo $HOME)/landing_on_moving_platform/src/training_q_learning/scripts/training/launch_vicon_bridge.sh $MAV_NAME $ROS_PORT $GAZ_PORT" &
sleep 2


echo "Launching vicon interface..."
screen -d -m -S "$(echo $SIM_ID)_interface" bash -i -c "$(echo $HOME)/landing_on_moving_platform/src/training_q_learning/scripts/training/launch_vicon_interface.sh $MAV_NAME $ROS_PORT $GAZ_PORT" &
sleep 2

echo "Launching vicon stability axes publisher..."
screen -d -m -S "$(echo $SIM_ID)_publish_stability_axes" bash -i -c "$(echo $HOME)/landing_on_moving_platform/src/training_q_learning/scripts/training/launch_vicon_publish_stability_axes.sh $MAV_NAME $ROS_PORT $GAZ_PORT" &
sleep 2

echo "Launching computation of relative information..."
screen -d -m -S "$(echo $SIM_ID)_relative_information_drone_moving_platform" bash -i -c "$(echo $HOME)/landing_on_moving_platform/src/training_q_learning/scripts/training/launch_vicon_compute_relative_information.sh $MAV_NAME $ROS_PORT $GAZ_PORT" &
sleep 2

echo "Launching observation generator..."
screen -d -m -S "$(echo $SIM_ID)_observation_generator" bash -i -c "$(echo $HOME)/landing_on_moving_platform/src/training_q_learning/scripts/training/launch_vicon_observation_generator.sh $MAV_NAME $ROS_PORT $GAZ_PORT" &
sleep 2

echo "Launching roll pitch yawrate thrust to uav pose conversion..."
screen -d -m -S "$(echo $SIM_ID)_action_to_uavpose_conversion" bash -i -c "$(echo $HOME)/landing_on_moving_platform/src/training_q_learning/scripts/training/launch_vicon_action_to_uavpose.sh $MAV_NAME $ROS_PORT $GAZ_PORT" &
sleep 2

echo "Launching state estimate frame publisher..."
screen -d -m -S "$(echo $SIM_ID)_publish_state_estimate_frame" bash -i -c "$(echo $HOME)/landing_on_moving_platform/src/training_q_learning/scripts/training/launch_vicon_publish_state_estimate_copter_frame.sh $MAV_NAME $ROS_PORT $GAZ_PORT" &
sleep 2


echo "Launching extraction of pose and twist from uav state estimate..."
screen -d -m -S "$(echo $SIM_ID)_pose_twist_extraction_from_state_estimate" bash -i -c "$(echo $HOME)/landing_on_moving_platform/src/training_q_learning/scripts/training/launch_vicon_uav_pose_twist_from_state_estimate.sh $MAV_NAME $ROS_PORT $GAZ_PORT" &
sleep 2
















