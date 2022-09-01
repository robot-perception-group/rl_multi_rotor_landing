#export ROS_MASTER_URI=http://192.168.10.2:11317
export ROS_IP=10.42.0.105
source /home/ubuntu/landing_on_moving_platform_copter/devel/setup.bash
sleep 10;
rosrun librepilot librepilot_node fc0 /dev/flightcontroller_serial 115200
