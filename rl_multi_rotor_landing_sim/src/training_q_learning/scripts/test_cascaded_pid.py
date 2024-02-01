'''
Script creates the simulation environment that is required in order to use  
4 separate cascaded PID controllers to independently control the longitudinal, 
lateral vertical and yaw motion of the UAV intended to land on a moving platform.
'''

from training_q_learning.custom_q_learning import QLearning
import rospy

#Initialize the parameters defining motion in longitudinal direction.
q_learning = QLearning()

#variables
node_name="landing_simulation_gym_node"
drone_name = rospy.get_param(rospy.get_namespace()+node_name+'/drone_name')
topic_prefix = '/'+drone_name+'/'

if __name__ == '__main__':
    #Initialize node
    rospy.init_node(q_learning.node_name, anonymous=True, log_level=rospy.INFO)

    # #Load training
    # q_learning.load_training()
   
    #Initialize training environment
    q_learning.init_training_env(2)


    #Get initial state
    #Handle longitudinal direction
    q_learning.env.reset()

    #Enter loop
    done = False
    #Set test_mode_activated to true
    q_learning.env.landing_simulation_object.test_mode_activated = True
    q_learning.env.test_mode_activated = True
    for _ in range(1000000):
        if done == True:
            #If a terminal condition has been reached, reset simulation
            current_state_lon = q_learning.env.reset()
        
        done = q_learning.env.external_controller_step(10)






    