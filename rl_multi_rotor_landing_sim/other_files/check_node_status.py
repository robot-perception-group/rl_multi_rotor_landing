# This script keeps track of all the nodes running and prints out information if one node failed
import rosnode
from datetime import datetime
import time


#Execute check loop
list_of_nodes_to_check = ['/command_moving_platform_trajectories_node','/moving_platform_joint_publisher_node', '/hummingbird/robot_state_publisher', '/moving_platform_state_publisher_node', '/hummingbird/joint_state_publisher', '/moving_platform_world_base_link_transform_publisher_node', '/gazebo_gui', '/hummingbird/pid_yaw/controller_yaw', '/hummingbird/stability_frame_publisher_node', '/hummingbird/compute_relative_state_moving_platform_drone_node', '/hummingbird/training_action_interface_node', '/hummingbird/roll_pitch_yawrate_thrust_controller_node', '/hummingbird/compute_observation_node', '/hummingbird/pid_v_z/controller_v_z', '/rosout', '/gazebo']
reported_nodes = []
while True:
    running_nodes = rosnode.get_node_names() 
    now = datetime.now()
    for node in list_of_nodes_to_check:
        if not node in running_nodes and node not in reported_nodes:
            print("\033[93mNode",node,"failed at",now.strftime("%H:%M:%S"))
            reported_nodes.append(node)
    time.sleep(10)


