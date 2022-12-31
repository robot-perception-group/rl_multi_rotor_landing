"""
This script kills all instances of ROS and GAZEBO as well as the associated screens
"""

#Killall roscore rosmaster gazebo gzserver gzclient
killall roscore rosmaster gazebo gzserver gzclient

#Kill associated screens
for sim in "sim_1" "sim_2" "sim_3" "sim_4" "sim_5" "sim_6" "sim_7" "sim_8" "sim_9" "sim_10" "sim_11" "sim_12" "sim_13" "sim_14" "sim_15" "sim_16" "sim_17" "sim_18"
do
    ./kill_sim.sh $sim
done


