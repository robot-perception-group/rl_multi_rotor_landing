#This script sets the necessary environment variables and sources the necessaary scripts to 
#have access to the ROS / Gazebo data of a specific training instance

ROS_PORT=$1
GAZ_PORT=$2
ROSIP=$(hostname -I | cut -d' ' -f1) 



#Source workspace
if [[ -z "${ROS_PROJECT_ROOT}" ]]
then
   echo "First run source your_downloaded_folder/rl_multi_rotor_landing_sim/other_files/setup.bash, then repeat!"
else
    export ROS_MASTER_URI=http://$ROSIP:$ROS_PORT;\
    export GAZEBO_MASTER_URI=http://$ROSIP:$GAZ_PORT;\
    export ROS_IP=$ROSIP;\

    echo "ROS_PORT = $ROS_PORT"
    echo "GAZEBO_PORT = $GAZ_PORT"
    echo "ROS_IP = $ROS_IP"
    echo "ROS_MASTER_URI=$ROS_MASTER_URI"
    echo "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
fi
