from training_q_learning.custom_q_learning import QLearning
from training_q_learning.utils_multiresolution import add_to_dict_refinement_min_max_values_for_state,get_state_grid_idx_from_ros_msg
from training_q_learning.parameters import Parameters
from std_msgs.msg import Float64,Float64MultiArray
from copy import deepcopy
import rospy
import numpy as np
from librepilot.msg import TransmitterInfo

M_PI = 3.14159265

#Initialize the parameters defining motion in longitudinal direction.
parameters_lon = Parameters()
q_learning_lon = QLearning()

#Initialize parameters definining motion in lateral direction.
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


#Set up variables
node_name="vicon_gym_node"
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name')
topic_prefix = '/'+drone_name+'/'
fc_name = rospy.get_param(rospy.get_namespace()+node_name+'/fc_name','fc0')
flight_activated = False
topic_prefix_fc = '/'+fc_name+'/'

#topic definition
current_state_idx_x_topic = (topic_prefix+'flight/current_idx_x',Float64MultiArray)
current_state_idx_y_topic = (topic_prefix+'flight/current_idx_y',Float64MultiArray)
flightmode_topic = (topic_prefix_fc+'TransmitterInfo',TransmitterInfo)


#Publishers
current_state_idx_x_publisher = rospy.Publisher(current_state_idx_x_topic[0],current_state_idx_x_topic[1],queue_size = 0)
current_state_idx_y_publisher = rospy.Publisher(current_state_idx_y_topic[0],current_state_idx_y_topic[1],queue_size = 0)

class FlightActivated():
    def __init__(self):
        self.flight_activated = False
        return
    def read_flightmode(self,msg):
        if msg.ROSControlled == 1:
            self.flight_activated = True
        else:
            self.flight_activated = False
        return

if __name__ == '__main__':

    #Initialize node
    rospy.init_node(q_learning_lon.node_name, anonymous=True, log_level=rospy.INFO)
    
    #Class init
    flight_activated = FlightActivated()

    #Subscribers
    flightmode_subscriber = rospy.Subscriber(flightmode_topic[0],flightmode_topic[1],flight_activated.read_flightmode)

    #Load training
    q_learning_lon.load_training()
   
    #Initialize training environment
    q_learning_lon.init_vicon_env(2)

    #Initialize dictionaries defining the discretization
    refinement_steps_dict_lon = {}
    refinement_steps_dict_lat = {}

    #Build the dictionaries defining the discretization.
    for state in parameters_lon.uav_parameters.observation_msg_strings.values(): 
        add_to_dict_refinement_min_max_values_for_state(state,refinement_steps_dict_lon)        
        assert len(refinement_steps_dict_lon[state]) == len(q_learning_lon.Q_table),"The number of discretization steps that are specified for state "+state+" is "+str(len(refinement_steps_dict_lon[state]))+" and does not match the expected number "+ str(q_learning_lon.Q_table.shape[0])
    q_learning_lon.refinement_steps_dict = deepcopy(refinement_steps_dict_lon)

    for state in parameters_lat.uav_parameters.observation_msg_strings.values(): 
        add_to_dict_refinement_min_max_values_for_state(state,refinement_steps_dict_lat)        
        assert len(refinement_steps_dict_lat[state]) == len(q_learning_lon.Q_table),"The number of discretization steps that are specified for state "+state+" is "+str(len(refinement_steps_dict_lat[state]))+" and does not match the expected number "+ str(q_learning_lon.Q_table.shape[0])

    print("shape Q table: ",q_learning_lon.Q_table.shape)
    print("flight_activated=",flight_activated.flight_activated)
    check_rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if flight_activated.flight_activated:
            #Get initial state
            print("Triggered reset...")
            current_state_lon = q_learning_lon.env.reset()
            print("Reset completed!")
            current_state_lon = q_learning_lon.env.convert_observation_msg( current_state_lon)
            current_state_lat = q_learning_lon.env.vicon_object.get_observation()

            q_learning_lon.env.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
            current_state_lat = q_learning_lon.env.convert_observation_msg(current_state_lat)

            q_learning_lon.env.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)

            current_state_idx_lon = get_state_grid_idx_from_ros_msg(current_state_lon[0],current_state_lon[1],refinement_steps_dict_lon,q_learning_lon.n_r,parameters_lon)
            current_state_idx_lat = get_state_grid_idx_from_ros_msg(current_state_lat[0],current_state_lat[1],refinement_steps_dict_lat,q_learning_lon.n_r,parameters_lat)
            print("current_state_idx_lon = ",current_state_idx_lon)
            print("current_state_idx_lat = ",current_state_idx_lat)
            
            done = False
            #Set test_mode_activated to true
            q_learning_lon.env.vicon_object.test_mode_activated = True
            #Enter loop
            for _ in range(100000):
                if done == True:
                    #If a terminal condition has been reached, reset simulation
                    print("touchdown current_idx:", current_state_idx_lon)
                    q_learning_lon.env.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
                    q_learning_lon.env.vicon_object.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
                    current_state_lon = q_learning_lon.env.reset()
                    q_learning_lon.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
                #Handle lateral motion
                q_learning_lon.env.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
                q_learning_lon.env.vicon_object.parameters.uav_parameters = deepcopy(parameters_lat.uav_parameters)
                current_state_lat = q_learning_lon.env.vicon_object.get_observation()
                current_state_lat = q_learning_lon.env.convert_observation_msg(current_state_lat)
                #Handle longitudinal motion
                q_learning_lon.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
                q_learning_lon.env.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
                q_learning_lon.env.vicon_object.parameters.uav_parameters = deepcopy(parameters_lon.uav_parameters)
                q_learning_lon.env.vicon_object.current_state_idx_lon = current_state_idx_lon
                q_learning_lon.env.vicon_object.n_r = parameters_lon.rl_parameters.n_r

                #Determine actions
                action_lon = q_learning_lon.predict_action(current_state_idx_lon,verbose = False)
                print("action_lon =",parameters_lon.uav_parameters.action_strings[action_lon])
                action_lat = q_learning_lon.predict_action(current_state_idx_lat,verbose = False)
                print("action_lat =",parameters_lat.uav_parameters.action_strings[action_lat])

                #Perform one training step
                current_state_idx_lon,current_state_idx_lat, done, _ = q_learning_lon.env.step_2D(action_lon,action_lat,parameters_lon,parameters_lat,refinement_steps_dict_lon,refinement_steps_dict_lat)

                print("current_state_idx_lon = ",current_state_idx_lon)
                print("current_state_idx_lat = ",current_state_idx_lat)

                #Publish data
                current_state_idx_x_msg = Float64MultiArray()
                current_state_idx_x_msg.data = current_state_idx_lon
                current_state_idx_x_publisher.publish(current_state_idx_x_msg)
                current_state_idx_y_msg = Float64MultiArray()
                current_state_idx_y_msg.data = current_state_idx_lat
                current_state_idx_y_publisher.publish(current_state_idx_y_msg)
        else:
            check_rate.sleep()


        