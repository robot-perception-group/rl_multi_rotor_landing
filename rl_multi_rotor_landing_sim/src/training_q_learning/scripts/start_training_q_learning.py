from training_q_learning.custom_q_learning import QLearning
from training_q_learning.parameters import Parameters
import rospy

q_learning = QLearning()
parameters = Parameters()
list_of_implemented_q_learning_algorithms = ["one_step_q_learning","multi_step_q_sigma_learning"]

if __name__ == '__main__':
####Init ros env
    rospy.init_node(q_learning.node_name, anonymous=True, log_level=rospy.INFO)

    q_learning.init_training_env(2)
    if parameters.rl_parameters.q_learning_algorithm == "double_q_learning":
        q_learning.initialize_table("Q_table",0.0,save_table = True)
        q_learning.initialize_table("Q_table_double",0.0,save_table = True)
        q_learning.initialize_table("state_action_counter",0.0,save_table = True)
    elif parameters.rl_parameters.q_learning_algorithm == "q_lambda_learning":
        q_learning.initialize_table("E",0.0,save_table = True)
        q_learning.initialize_table("Q_table",0.0,save_table = True)
        q_learning.initialize_table("state_action_counter",0.0,save_table = True)

    else:
        q_learning.initialize_table("Q_table",0.0,save_table = True)
        q_learning.initialize_table("state_action_counter",0.0,save_table = True)
    
       

    # q_learning.add_new_refinement_step_at_end()
    q_learning.setup_logging_system()

    if parameters.rl_parameters.q_learning_algorithm == "one_step_q_learning":
        q_learning.one_step_q_learning()
    elif parameters.rl_parameters.q_learning_algorithm == "multi_step_q_sigma_learning":
        q_learning.multi_step_q_sigma_learning()
    elif parameters.rl_parameters.q_learning_algorithm == "double_q_learning":
        q_learning.double_q_learning()
    elif parameters.rl_parameters.q_learning_algorithm == "q_lambda_learning":
        q_learning.q_lambda_learning()
    else:
        rospy.logwarn("Selected Q-Learning algorithm '"+parameters.rl_parameters.q_learning_algorithm+"' is not implemented. Currently implemented is: "+", ".join(list_of_implemented_q_learning_algorithms,)+". Aborting...")













