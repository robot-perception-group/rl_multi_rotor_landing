#This script sets the necessary environment variables and sources the appropriate files to run the ros package

#Get initial folder where script is called from
initial_wd=`pwd`

#Navigate into the folder where this script is stored 
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd $parent_path

#source catkin workspace
source ../devel/setup.bash

#Export ros project root dir
cd ..
ros_project_root_dir=`pwd`
export ROS_PROJECT_ROOT=$ros_project_root_dir
echo "Set ROS_PROJECT_ROOT=$(echo $ROS_PROJECT_ROOT)"

#Update ROS package path, so that only the package sourced above is used
export ROS_PACKAGE_PATH=$ros_project_root_dir/src:/opt/ros/noetic/share
cd $initial_wd 