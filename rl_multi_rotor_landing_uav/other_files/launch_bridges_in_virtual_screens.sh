#!/bin/bash
echo "Killing existing screen sessions if existing..."
killall screen
sleep 1



#launch telemetry bridge
echo "Launching telemetry bridge in virtual screen..."
screen -d -m -S "telemetry_bridge" bash -i -c "/home/ubuntu/landing_on_moving_platform_copter/other_files/launch_telemetry_bridge.sh"

#launch ros core
echo "Launching ROS core  in virtual screen..."
screen -d -m -S "ros_core" bash -i -c "/home/ubuntu/landing_on_moving_platform_copter/other_files/launch_ros_core.sh"

#launch ros bridge
echo "Launching ROS bridge in virtual screen..."
screen -d -m -S "ros_bridge" bash -i -c "/home/ubuntu/landing_on_moving_platform_copter/other_files/launch_ros_bridge.sh"






