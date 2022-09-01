from training_q_learning.custom_q_learning import QLearning
from training_q_learning.parameters import Parameters
import rospy

parameters = Parameters()
q_learning = QLearning()





if __name__ == '__main__':
    # Init the training environment
    rospy.init_node(q_learning.node_name, anonymous=True, log_level=rospy.INFO)

    #Load the training
    q_learning.load_training()

    #Add at least one new curriculum step
    if parameters.rl_parameters.add_refinement_step == 0:
        pass
    elif parameters.rl_parameters.add_refinement_step > 0 and isinstance(parameters.rl_parameters.add_refinement_step,int):
        q_learning.add_multiple_refinement_steps_at_end(parameters.rl_parameters.add_refinement_step)
    else:
        print("Value specified for add_ref_step in the parameter file is not valid. ABORT...")
        exit()
    
    #Reset the values of tables if specified.
    if len(parameters.rl_parameters.reset_table_values):
        for item in parameters.rl_parameters.reset_table_values.items():
            q_learning.reset_table_values_to_value(item[0],item[1])
            
    #Initialize the training environment        
    q_learning.init_training_env(2)
    q_learning.setup_logging_system()
    if parameters.rl_parameters.q_learning_algorithm == "double_q_learning":
        q_learning.double_q_learning()
    else:
        rospy.logwarn("Selected Q-Learning algorithm '"+parameters.rl_parameters.q_learning_algorithm+"' is not implemented. Currently implemented is only Double Q-Learning. Aborting...")


