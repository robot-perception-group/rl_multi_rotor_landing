ID_TAG=$1

#get time when script is called
t_start=$(date +"%Y%m%d_%H%M%S")

file_name="$(echo $ID_TAG)_$(echo $t_start)"
echo "Recording to file $file_name.bag"
#launch rosbag
rosbag record -O $(echo $file_name) -a


