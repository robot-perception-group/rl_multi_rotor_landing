#!/bin/sh

if ! which catkin_init_workspace >/dev/null; then
    echo "You need to initialize your ROS environment before calling this script."
    echo "source /opt/ros/<ROSVERSION>/setup.bash"
    exit
fi

for dir in rl_multi_rotor_landing_gcs rl_multi_rotor_landing_uav rl_multi_rotor_landing_sim; do
    cd $dir/src
    catkin_init_workspace
    cd ../..
done

echo "Workspaces initialized successfully."
