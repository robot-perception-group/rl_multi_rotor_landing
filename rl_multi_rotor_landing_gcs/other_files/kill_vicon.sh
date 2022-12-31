SIM_ID=$1

#kill screen sessions
screen -S "$(echo $SIM_ID)_landing_simulation" -p 0 -X quit
screen -S "$(echo $SIM_ID)_publish_stability_axes" -p 0 -X quit
screen -S "$(echo $SIM_ID)_relative_information_drone_moving_platform" -p 0 -X quit
screen -S "$(echo $SIM_ID)_observation_generator" -p 0 -X quit
screen -S "$(echo $SIM_ID)_action_to_uavpose_conversion" -p 0 -X quit
screen -S "$(echo $SIM_ID)_interface" -p 0 -X quit
screen -S "$(echo $SIM_ID)_bridge" -p 0 -X quit
screen -S "$(echo $SIM_ID)_pose_twist_extraction_from_state_estimate" -p 0 -X quit
screen -S "$(echo $SIM_ID)_publish_state_estimate_frame" -p 0 -X quit


echo "All screens of simulation '$(echo $SIM_ID)' killed..."
echo "List of remaining screens:"
screen -list
echo "Gazebo might still be running and require additional time to shut down"

