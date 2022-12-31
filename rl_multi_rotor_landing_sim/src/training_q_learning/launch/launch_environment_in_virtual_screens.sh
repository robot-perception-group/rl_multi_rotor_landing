#Script launches landing simulation in virtual screens

SIM_ID=$1
MAV_NAME=$2
ROS_PORT=$3 
GAZ_PORT=$4
ROSIP=$(hostname -I | cut -d' ' -f1) 

export ROS_MASTER_URI=http://$ROSIP:$ROS_PORT;\
export GAZEBO_MASTER_URI=http://$ROSIP:$GAZ_PORT;\
export ROS_IP=$ROSIP;\

echo "ROS_PORT = $ROS_PORT"
echo "GAZEBO_PORT = $GAZ_PORT"
echo "ROS_IP = $ROS_IP"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"

screen -d -m -S "$(echo $SIM_ID)_landing_simulation" bash -i -c "$(echo $ROS_PROJECT_ROOT)/src/training_q_learning/launch/launch_landing_simulation.sh $MAV_NAME $ROS_PORT $GAZ_PORT"&
echo "Launched landing_simulation. Waiting 10 seconds to ensure proper start up..."
sleep 10

screen -d -m -S "$(echo $SIM_ID)_publish_stability_axes" bash -i -c "$(echo $ROS_PROJECT_ROOT)/src/training_q_learning/launch/launch_publish_stability_axes.sh $MAV_NAME $ROS_PORT $GAZ_PORT" &
echo "Launched stability axes publisher."

screen -d -m -S "$(echo $SIM_ID)_relative_information_drone_moving_platform" bash -i -c "$(echo $ROS_PROJECT_ROOT)/src/training_q_learning/launch/launch_compute_relative_information.sh $MAV_NAME $ROS_PORT $GAZ_PORT" &
echo "Launched computation of relative information between drone and moving platform."

screen -d -m -S "$(echo $SIM_ID)_observation_generator" bash -i -c "$(echo $ROS_PROJECT_ROOT)/src/training_q_learning/launch/launch_observation_generator.sh $MAV_NAME $ROS_PORT $GAZ_PORT" &
echo "Launched observation generator."

screen -d -m -S "$(echo $SIM_ID)_training_action_interface" bash -i -c "$(echo $ROS_PROJECT_ROOT)/src/training_q_learning/launch/launch_action_interface.sh $MAV_NAME $ROS_PORT $GAZ_PORT" &
echo "Launched training action interface."
















