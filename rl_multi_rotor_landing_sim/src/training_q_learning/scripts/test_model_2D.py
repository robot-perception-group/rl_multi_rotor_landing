'''
Script uses two instances of one RL agent to control the movement of the drone in longitudinal and lateral direction.
'''

from training_q_learning.custom_q_learning import QLearning
from training_q_learning.utils_multiresolution import add_cur_step_lims_of_state,get_discrete_state_from_ros_msg
from training_q_learning.parameters import Parameters
from std_msgs.msg import Float64MultiArray
from copy import deepcopy
import rospy

#Initialize the parameters defining motion in longitudinal direction.
parameters_lon = Parameters()
q_learning_lon = QLearning()

#Initialize parameters definining motion in lateral direction. The same values are used as for the longitudinal direction assuming a symmetric copter.
parameters_lat = Parameters()
parameters_lat.uav_parameters.action_strings = {0:"increase_roll",
                                                1:"decrease_roll",
                                                2:"do_nothing"
                                                }
parameters_lat.uav_parameters.action_max_values = {"roll":parameters_lon.uav_parameters.action_max_values["pitch"]}
parameters_lat.uav_parameters.action_delta_values = {"roll":parameters_lon.uav_parameters.action_delta_values["pitch"]} 
parameters_lat.uav_parameters.observation_msg_strings = {0:"rel_p_y",
                                                         1:"rel_v_y",
                                                         2:"rel_a_y"} 
parameters_lat.uav_parameters.observation_max_values = {"rel_p_y":parameters_lon.uav_parameters.observation_max_values["rel_p_x"], 
                                                        "rel_v_y":parameters_lon.uav_parameters.observation_max_values["rel_v_x"],
                                                        "rel_a_y":parameters_lon.uav_parameters.observation_max_values["rel_a_x"]}                                                         

#variables
node_name="landing_simulation_gym_node"
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name')
topic_prefix = '/'+drone_name+'/'

#Topics
current_state_idx_x_topic = (topic_prefix+'training/current_idx_x',Float64MultiArray)
current_state_idx_y_topic = (topic_prefix+'training/current_idx_y',Float64MultiArray)

#Publishers
current_state_idx_x_publisher = rospy.Publisher(current_state_idx_x_topic[0],current_state_idx_x_topic[1],queue_size = 0)
current_state_idx_y_publisher = rospy.Publisher(current_state_idx_y_topic[0],current_state_idx_y_topic[1],queue_size = 0)

if __name__ == '__main__':
    #Initialize node
    rospy.init_node(q_learning_lon.node_name, anonymous=True, log_level=rospy.INFO)

    #Load training
    q_learning_lon.load_training()
   
    #Initialize training environment
    q_learning_lon.init_training_env(2)

    #Initialize dictionaries defining the discretization
    lims_of_cur_steps_lon = {}
    lims_of_cur_steps_lat = {}

    #Build the dictionaries defining the discretization.
    #Longitudinal direction
    for state in parameters_lon.uav_parameters.observation_msg_strings.values(): 
        add_cur_step_lims_of_state(state,lims_of_cur_steps_lon)        
        assert len(lims_of_cur_steps_lon[state]) == len(q_learning_lon.Q_table),"The number of discretization steps that are specified for state "+state+" is "+str(len(lims_of_cur_steps_lon[state]))+" and does not match the expected number "+ str(q_learning_lon.Q_table.shape[0])
    q_learning_lon.lims_of_cur_steps = deepcopy(lims_of_cur_steps_lon)

    #Lateral direction
    for state in parameters_lat.uav_parameters.observation_msg_strings.values(): 
        add_cur_step_lims_of_state(state,lims_of_cur_steps_lat)        
        assert len(lims_of_cur_steps_lat[state]) == len(q_learning_lon.Q_table),"The number of discretization steps that are specified for state "+state+" is "+str(len(lims_of_cur_steps_lat[state]))+" and does not match the expected number "+ str(q_learning_lon.Q_table.shape[0])

    print("shape Q table: ",q_learning_lon.Q_table.shape)

    #Get initial state
    #Handle longitudinal direction
    current_state_lon = q_learning_lon.env.reset()
    current_state_lon = q_learning_lon.env.convert_observation_msg( current_state_lon)

    #Handle lateral direction
    #Get latest observations
    current_state_lat = q_learning_lon.env.landing_simulation_object.get_observation()

    #Replace the states with the lateral ones to apply methods of q_learning also for lateral states
    q_learning_lon.env.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)

    #Apply normalization
    current_state_lat = q_learning_lon.env.convert_observation_msg(current_state_lat)

    #Switch back to original parameters of longitudinal direction
    q_learning_lon.env.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)

    #Get the discrete states
    current_state_idx_lon = get_discrete_state_from_ros_msg(current_state_lon[0],current_state_lon[1],lims_of_cur_steps_lon,q_learning_lon.n_r,parameters_lon)
    current_state_idx_lat = get_discrete_state_from_ros_msg(current_state_lat[0],current_state_lat[1],lims_of_cur_steps_lat,q_learning_lon.n_r,parameters_lat)
    print("current_state_idx_lon = ",current_state_idx_lon)
    print("current_state_idx_lat = ",current_state_idx_lat)
    
    done = False
    #Set test_mode_activated to true
    q_learning_lon.env.landing_simulation_object.test_mode_activated = True
    q_learning_lon.env.test_mode_activated = True

    #Enter loop
    for _ in range(100000):

        if done == True:
            #If a terminal condition has been reached, reset simulation
            # print("touchdown current_idx:", current_state_idx_lon)
            q_learning_lon.env.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
            q_learning_lon.env.landing_simulation_object.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
            current_state_lon = q_learning_lon.env.reset()
             
        #Handle lateral motion
        q_learning_lon.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
        q_learning_lon.env.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
        q_learning_lon.env.landing_simulation_object.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
        current_state_lat = q_learning_lon.env.landing_simulation_object.get_observation()
        current_state_lat = q_learning_lon.env.convert_observation_msg(current_state_lat)
        
        #Handle longitudinal motion
        q_learning_lon.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
        q_learning_lon.env.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
        q_learning_lon.env.landing_simulation_object.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
        q_learning_lon.env.landing_simulation_object.current_state_idx_lon = current_state_idx_lon
        q_learning_lon.env.landing_simulation_object.n_r = parameters_lon.rl_parameters.n_r

        #Determine actions
        action_lon = q_learning_lon.predict_action(current_state_idx_lon,verbose = False)
        action_lat = q_learning_lon.predict_action(current_state_idx_lat,verbose = False)

        #Perform one training step
        current_state_idx_lon,current_state_idx_lat, done, _ = q_learning_lon.env.step_2D(action_lon,action_lat,parameters_lon,parameters_lat,lims_of_cur_steps_lon,lims_of_cur_steps_lat)
        if parameters_lon.rl_parameters.verbose:
            print("current_state_idx_lon = ",current_state_idx_lon)
            print("action_lon =",parameters_lon.uav_parameters.action_strings[action_lon])
            print("current_state_idx_lat = ",current_state_idx_lat)
            print("action_lat =",parameters_lat.uav_parameters.action_strings[action_lat])

            
        #Publish data
        current_state_idx_x_msg = Float64MultiArray()
        current_state_idx_x_msg.data = current_state_idx_lon
        current_state_idx_x_publisher.publish(current_state_idx_x_msg)
        current_state_idx_y_msg = Float64MultiArray()
        current_state_idx_y_msg.data = current_state_idx_lat
        current_state_idx_y_publisher.publish(current_state_idx_y_msg)

        