"""
This script can be used to plot the frequencies of the ROS topics that were logged during the training procedure
"""

import matplotlib.pyplot as plt
import pandas as pd
import os
import rospy

## Input
log_file_path = '/home/pgoldschmid/rl_multiple_landing/sim_3/test_landing/rl_multi_rotor_landing_sim/src/training_q_learning/training_results/training_q_learning_1/logged_topic_freqs.csv'
plot_topics = [
 "/clock",
"/gazebo/link_states",
"/gazebo/model_states",
"/gazebo/parameter_descriptions",
"/gazebo/parameter_updates",
"/gazebo/performance_metrics",
 "/gazebo/set_link_state",
 "/gazebo/set_model_state",
 "/hummingbird/command/motor_speed",
 "/hummingbird/command/roll_pitch_yawrate_thrust",
 "/hummingbird/gazebo/command/motor_speed",
 "/hummingbird/ground_truth/imu",
 "/hummingbird/ground_truth/odometry",
 "/hummingbird/ground_truth/pose",
 "/hummingbird/ground_truth/pose_with_covariance",
 "/hummingbird/ground_truth/position",
 "/hummingbird/ground_truth/transform",
 "/hummingbird/imu",
 "/hummingbird/joint_states",
 "/hummingbird/landing_simulation/drone/debug_target_frame/roll_pitch_yaw",
 "/hummingbird/landing_simulation/drone/state",
 "/hummingbird/landing_simulation/moving_platform/debug_target_frame/roll_pitch_yaw",
 "/hummingbird/landing_simulation/moving_platform/state",
 "/hummingbird/landing_simulation/relative_moving_platform_drone/debug_target_frame/roll_pitch_yaw",
 "/hummingbird/landing_simulation/relative_moving_platform_drone/state/pose",
 "/hummingbird/landing_simulation/relative_moving_platform_drone/state/twist",
 "/hummingbird/landing_simulation/world_frame/drone/state",
 "/hummingbird/landing_simulation/world_frame/moving_platform/state",
 "/hummingbird/motor_speed",
 "/hummingbird/motor_speed/0",
 "/hummingbird/motor_speed/1",
 "/hummingbird/motor_speed/2",
 "/hummingbird/motor_speed/3",
 "/hummingbird/odometry_sensor1/odometry",
 "/hummingbird/odometry_sensor1/pose",
 "/hummingbird/odometry_sensor1/pose_with_covariance",
 "/hummingbird/odometry_sensor1/position",
 "/hummingbird/odometry_sensor1/transform",
"/hummingbird/pid_v_z/controller_v_z/parameter_descriptions",
"/hummingbird/pid_v_z/controller_v_z/parameter_updates",
"/hummingbird/pid_v_z/pid_debug",
 "/hummingbird/pid_v_z/pid_enable",
"/hummingbird/pid_yaw/controller_yaw/parameter_descriptions",
"/hummingbird/pid_yaw/controller_yaw/parameter_updates",
"/hummingbird/pid_yaw/pid_debug",
"/hummingbird/pid_yaw/pid_enable",
"/hummingbird/training/current_idx",
"/hummingbird/training/current_idx_x",
"/hummingbird/training/current_idx_y",
"/hummingbird/training/episode_cum_reward",
 "/hummingbird/training/init_reset_simulation",
 "/hummingbird/training/reset_simulation",
 "/hummingbird/training/reward",
 "/hummingbird/training/reward_action",
 "/hummingbird/training/reward_rel_pos",
 "/hummingbird/training/reward_rel_vel",
 "/hummingbird/training/step/execution_frequency",
 "/hummingbird/training/step/time_step_error",
 "/hummingbird/training_action_interface/action_to_interface",
 "/hummingbird/training_action_interface/control_effort/v_z",
 "/hummingbird/training_action_interface/control_effort/yaw",
 "/hummingbird/training_action_interface/setpoint/v_z",
 "/hummingbird/training_action_interface/setpoint/yaw",
 "/hummingbird/training_action_interface/state/v_z",
 "/hummingbird/training_action_interface/state/yaw",
 "/hummingbird/training_observation_interface/observations",
 "/hummingbird/training_observation_interface/observations_actions",
"/hummingbird/wind_speed",
"/joint_states",
"/moving_platform/commanded/pose",
"/moving_platform/contact",
 "/moving_platform/setpoints/trajectory_radius",
 "/moving_platform/setpoints/trajectory_radius_lateral",
 "/moving_platform/setpoints/trajectory_speed",
 "/moving_platform/setpoints/trajectory_speed_lateral",
"/moving_platform_joints",
 "/rosout",
"/rosout_agg",
 "/tf",
"/tf_static",
]

## Calc
#Load csv
#Create data dict containing the logged data as pd data frames
datas = dict()
data_lengths = []
df = pd.read_csv(log_file_path,skiprows = 0,delimiter = ',') 

data_dict = dict()
for topic in plot_topics:
    if len(df.loc[df["topic"] == topic]):
        data_dict[topic] = df.loc[df["topic"] == topic]

## Vis
i = 0
for topic in data_dict.keys():
    plt.figure()
    plt.ion()
    x = (data_dict[topic]["time"]-data_dict[topic]["time"].iloc[0])/10**9
    y = data_dict[topic]["avg_rate"]
    plt.plot(x,y,label=topic)

    plt.title("Topic Frequency " + topic)
    plt.xlabel("Time [s]")
    plt.ylabel("Topic Frequency [hz]")
    plt.grid()
    plt.legend()
    save_path = os.path.join("/home/pgoldschmid/Desktop/topic_freqs",topic.replace("/","_"))
    plt.savefig(save_path,format = "png")
    i += 1



