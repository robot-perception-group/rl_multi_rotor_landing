SIM_ID=$1


#kill screen sessions
screen -S "$(echo $SIM_ID)_landing_simulation" -p 0 -X quit
screen -S "$(echo $SIM_ID)_publish_stability_axes" -p 0 -X quit
screen -S "$(echo $SIM_ID)_relative_information_drone_moving_platform" -p 0 -X quit
screen -S "$(echo $SIM_ID)_observation_generator" -p 0 -X quit
screen -S "$(echo $SIM_ID)_training_action_interface" -p 0 -X quit
screen -S "$(echo $SIM_ID)_cascaded_pid_x" -p 0 -X quit
screen -S "$(echo $SIM_ID)_cascaded_pid_y" -p 0 -X quit
screen -S "$(echo $SIM_ID)_cascaded_pid_z" -p 0 -X quit
screen -S "$(echo $SIM_ID)_cascaded_pid_interface" -p 0 -X quit

echo "All screens of simulation '$(echo $SIM_ID)' killed..."
echo "List of remaining screens:"
screen -list
echo "Gazebo might still be running and require additional time to shut down"

