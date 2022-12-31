SIM_ID=$1


#kill screen sessions
screen -S "$(echo $SIM_ID)_landing_simulation" -p 0 -X quit
screen -S "$(echo $SIM_ID)_publish_stability_axes" -p 0 -X quit
screen -S "$(echo $SIM_ID)_relative_information_drone_moving_platform" -p 0 -X quit
screen -S "$(echo $SIM_ID)_observation_generator" -p 0 -X quit
screen -S "$(echo $SIM_ID)_training_action_interface" -p 0 -X quit
screen -S "$(echo $SIM_ID)_analysis" -p 0 -X quit
screen -S "$(echo $SIM_ID)_vmp_x_setpoint" -p 0 -X quit
screen -S "$(echo $SIM_ID)_vmp_y_setpoint" -p 0 -X quit
screen -S "$(echo $SIM_ID)_test_model" -p 0 -X quit

echo "All screens of simulation '$(echo $SIM_ID)' killed..."
echo "List of remaining screens:"
screen -list
echo "Gazebo might still be running and require additional time to shut down"

