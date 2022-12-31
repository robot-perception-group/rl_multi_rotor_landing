'''
This script starts a training session.
'''
from training_q_learning.custom_q_learning import QLearning
from training_q_learning.parameters import Parameters
import rospy
import roslaunch
import time

list_of_implemented_q_learning_algorithms = ["double_q_learning"]

# Initialize class variables
parameters = Parameters()
q_learning = QLearning()

if __name__ == '__main__':
    #Check if selected q learning algorithm is implemented.
    if not parameters.rl_parameters.q_learning_algorithm in list_of_implemented_q_learning_algorithms:
        print("\033[91m The selected learning algorithm",parameters.rl_parameters.q_learning_algorithm,"is not implemented. List of currently implemented algorithms",list_of_implemented_q_learning_algorithms,". Aborting...")
        exit()

    # Init the node
    rospy.init_node(q_learning.node_name, anonymous=True, log_level=rospy.INFO)

    #Prepare training
    if parameters.rl_parameters.number_new_curriculum_steps == 0 and not parameters.rl_parameters.load_data_from and parameters.rl_parameters.proceed_from_last_episode == False: 
        #Init training env
        q_learning.init_training_env(2)

        #Initialize the required Q-tables
        if parameters.rl_parameters.q_learning_algorithm == "double_q_learning":
            q_learning.initialize_table("Q_table",0.0,save_table = True)
            q_learning.initialize_table("Q_table_double",0.0,save_table = True)
            q_learning.initialize_table("state_action_counter",0.0,save_table = True)
        else:
            q_learning.initialize_table("Q_table",0.0,save_table = True)
            q_learning.initialize_table("state_action_counter",0.0,save_table = True)

    elif parameters.rl_parameters.number_new_curriculum_steps == 0 and parameters.rl_parameters.load_data_from and parameters.rl_parameters.proceed_from_last_episode == True:
        #Load the training
        q_learning.load_training()

        #Reset the values of tables if specified.
        if len(parameters.rl_parameters.reset_table_values):
            for item in parameters.rl_parameters.reset_table_values.items():
                q_learning.reset_table_values_to_value(item[0],item[1])
                
        #Initialize the training environment        
        q_learning.init_training_env(2)

    elif parameters.rl_parameters.number_new_curriculum_steps > 0 and isinstance(parameters.rl_parameters.number_new_curriculum_steps,int) and parameters.rl_parameters.load_data_from and parameters.rl_parameters.proceed_from_last_episode == False:
        #Load the training
        q_learning.load_training()

        #Add new curriculum steps
        q_learning.append_multiple_curriculum_steps_to_curriculum_sequence(parameters.rl_parameters.number_new_curriculum_steps)

        #Reset the values of tables if specified.
        if len(parameters.rl_parameters.reset_table_values):
            for item in parameters.rl_parameters.reset_table_values.items():
                q_learning.reset_table_values_to_value(item[0],item[1])
                
        #Initialize the training environment        
        q_learning.init_training_env(2)

    else:
        print("\033[91mSelected parameters do not allow starting the training. ABORT...")
        exit()
    
    #Setup the logging system
    q_learning.setup_logging_system()

    #Begin the training process
    if parameters.rl_parameters.q_learning_algorithm == "double_q_learning":
        #Launch launcserver for launching additional nodes for monitoring the training
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        #Launch a node that copies the parameters used for the training to the training_results destination
        print("Initiating copying of parameters for logging...")
        node_copy = roslaunch.core.Node("training_q_learning","copy_parameters.py",name="copy_parameters")
        process_copy = launch.launch(node_copy)
        print("Checking if node for copying parameters is running...",process_copy.is_alive())

        #Launch a node that logs the topic frequencies that are used by the nodes to copy data
        print("Launching logging of topic frequencies...")
        node_log = roslaunch.core.Node("training_q_learning","log_topic_freqs.py",name="log_topic_freqs_node")
        process_log = launch.launch(node_log)
        print("Checking if logging node for topic frequencies is running...",process_log.is_alive())
        print("================== BEGIN OF TRAINING OF CURRICULUM STEP ===============")
        q_learning.double_q_learning()
        print("================== END OF TRAINING OF CURRICULUM STEP ===============")
        print("Terminating logging of topic frequncies...")
        process_log.stop()
        print("Checking if logging node for topic frequencies is running...",process_log.is_alive())
        
    else:
        rospy.logwarn("Selected Q-Learning algorithm '"+parameters.rl_parameters.q_learning_algorithm+"' is not implemented. Currently implemented is only Double Q-Learning. Aborting...")


