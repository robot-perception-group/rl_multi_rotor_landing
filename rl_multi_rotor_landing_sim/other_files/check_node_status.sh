# This script keeps track of all the nodes running and prints out information if one node failed


ROS_PORT=$1
GAZ_PORT=$2
source $(echo $ROS_PROJECT_ROOT)/other_files/prepare_terminal_window.sh $ROS_PORT $GAZ_PORT
python3 $(echo $ROS_PROJECT_ROOT)/other_files/check_node_status.py
