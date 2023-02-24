'''
This script contains useful functions that are required to set up the multi-resolution discretization approach, the different tables such 
as Q-tables required for the training procedure as well as all functions necessary for accessing the tables by indexing.
'''
import numpy as np
from training_q_learning.parameters import Parameters
from training_q_learning.msg import Action, ObservationRelativeState


#General variables
implemented_states_list = ["rel_p_x","rel_p_y","rel_v_x","rel_v_y","rel_a_x","rel_a_y"]

def initialize_grid_list(grid:list,init_value:float):
    '''
        The grid list can be initialized using this function. It will create a nested list according to the size of the different grid vectors.
        The grid vectors are the coordinate vectors of the space spanned by the grid vectors.
        The nesting is applied in such a way, that the indexing of the nested grid_list can be done as follows (g0,g1,g2 are the grid vectors):
        sub_list = grid_list[g0_i][g1_j][g2_k]...
    '''
    #Construct the nested list.
    grid_list = init_value
    for i in range(len(grid)-1,-1,-1):
        len_tmp = len(grid[i])
        grid_list = np.array([grid_list]*len_tmp)
    return grid_list
    
def discretize_relative_state(state_name:str,value:float,min_value:float,max_value:float,n_r:int,ratio:float):
    '''
    Function maps a continuous relative state value to the associated discrete state value idx. 
    The mapping is performed as follows.
            - The parameter ratio defines the proportion by which the absolute  max_value / min_value is reduced / increased in order one obtain the positive / negative goal value (the value of 
    the border of the goal interval). 
            -The range betweeen the positive / negative goal value and the max_value / min_value is then intersected with additional boundaries separating different intervals. 
            The number of additional boundaries is computed in such a way that the total number of intervals between min_value and max_value is n_r. The range between the positive / negative goal value and the
    max_value / min_value is intersected in intervals of equal size. Note that this interval size does not necessarily correspond to the size of the goal interval (range between negative and positive goal value).
    '''
    assert max_value >= min_value,"The max. value needs to be greater or equal than the min. value. Current min. value: "+str(min_value)+", current max. value "+str(max_value)
    assert n_r>0,"n_r must be greater than zero (currently n_r = "+str(n_r)+")"
    assert value <= max_value,"value must be less or equal to maximum value (value = "+str(value)+" and maximum value = "+str(max_value)+")"
    assert value >= min_value,"value must be greater or equal to minimum value (value ="+str(value)+" and minimum value ="+str(min_value)+")"
    assert state_name in implemented_states_list,"State "+state_name+" not in list of implemented states. Implemented states: "+", ".join(implemented_states_list)
    assert int(n_r+1) % 2 == 0,"n_r needs to be an odd number. Even numbers are not implemented yet. "+str(int((n_r+1)/2) % 2)
    
    #Determine the positive goal value
    pos_border_goal_value = ratio*max_value

    #Intersect the range from pos goal value to max value in such a way that the total sum of intervals between min value and max value is still n_r
    pos_half_value_range = np.linspace(pos_border_goal_value,max_value,int((n_r+1)/2))

    #Determine the negative border of the goal grid idx
    neg_border_goal_value = -pos_border_goal_value

    #Intersect the range from min value to negative goal value in such a way that the total sum of intervals between min value and max value is still n_r
    neg_half_value_range = np.linspace(min_value,neg_border_goal_value,int((n_r+1)/2))

    #Create an array consisting of the intervals
    value_range = np.concatenate([neg_half_value_range,pos_half_value_range])


    #Determine the idx of the interval in which the value resides. np.argmax returns always first occurence if several identical max values exist.
    idx = np.argmax(value_range >= value) -1
    
    #Consider boundary cases
    if value == min_value:
        idx = 0
    if value == max_value:
        idx = n_r -1
    if value == neg_border_goal_value:
        idx = int((n_r-1)/2)
    if value == pos_border_goal_value:
        idx = int((n_r-1)/2)

    #If a negative idx should be determined, print the case for which it happend
    if idx < 0:
        print("Detected negative value for discrete state", state_name)
        print("Values leading to negative discrete state value:")
        print("continuous value =",value)
        print("min_value =",min_value)
        print("max_value =",max_value)
        print("disctrete value =",idx)   

    return idx
    
def add_cur_step_lims_of_state(state:str,data_dict:dict,parameters:Parameters = None):
    '''Function reads the limit value for the different discretization steps defined for state "state" in the parameters file. 
    It uses these values to construct a dictionary that contains the positive and negative limit values for each state up to the curriculum step that was specified in the parameters file.
    The order of the list of positive and negative limit values is such that the limit values become smaller the higher the index.
    The resulting dictionary is returned.'''
    if not parameters:
        parameters = Parameters()
    else:
        parameters = parameters
    minmax_array = np.array([]).reshape(0,2)
    #Check if a list of discretization steps exist for the specified state
    if state in implemented_states_list:
        discretization_steps = np.flip(np.array(parameters.rl_parameters.discretization_steps[state]))

        #assert validity
        assert sorted(list(discretization_steps)) == list(discretization_steps),'The discretization steps defined for state "'+ state + '" are not sorted in descending order.'

        #build the return array
        i = 0
        for step in discretization_steps:            
            if i >= (len(discretization_steps)-1) - (parameters.rl_parameters.curriculum_step):
                minmax_array = np.insert(minmax_array,0,[-step,step],axis = 0)
            i += 1
        data_dict[state] = minmax_array
    return minmax_array

def get_latest_cur_step_for_state(value:float,state:str,lims_of_cur_steps:dict):
    '''
    Function determines the idx of the latest curriculum step in the sequential curriculum whose relative state limit values define a range in which the current relative state value resides.
    If the continuous value is equal to a limit value of one of the possible curriculum steps, the idx of the later added curriculum step is returned.
    The counting of the curriculum step indices starts with zero.
    If the value does not fall into the range spanned by the limit values of any curriculum step, the initial curriculum step idx (cur_step_idx = 0) is returned and a warning is printed.
    '''
    lim_array = lims_of_cur_steps[state]

    #Validity check
    assert sorted(list(lim_array[:,1]),reverse = True) == list(lim_array[:,1]),'The min and max limit values are not sorted in descending order for increasing index of curriculum steps.'
    
    #Count curriculum steps that include the value in the allowed value range
    cur_step_idx = 0
    for i in range(len(lim_array)):
        if lim_array[(i,0)] <= value <= lim_array[(i,1)]:
            cur_step_idx += 1
    
    #Subtract 1 to consider indexing starting with zero
    cur_step_idx = cur_step_idx - 1 

    #If the value is outside any range provided by the minmax_array
    if cur_step_idx < 0:
        cur_step_idx = 0
        print('\033[93m'+'Detected value = '+str(value)+' outside specified limits for state', state,'for all curriculum steps. Will return curriculum step idx 0.' + '\033[0m')
    return cur_step_idx
    
    

def get_discrete_rel_states_from_ros_msg(msg:ObservationRelativeState,lims_of_cur_steps:dict,n_r:int,parameters:Parameters):
    ''' 
        Function maps the continouos values of the relative position, relative velocity and relative acceleration (the relative states) to the discrete state values. 
        These indices comply with the following conditions.
            - For each continouos value of a relative state, the function determines the index of the discrete state (there are n_r discrete states in total for each continouos state) that is associated with the latest curriculum step that can be applied to all relatve states.
            - The latest curriculum step is the curriculum step for which all state values in "msg" are framed by the limit values associdated with that curriculum step.
        The function returns a dictionary. The keys are the used message field names, the values are the indices belonging to that message.
    '''    
    idx_dict = {}

    #Determine the latest refinement step of all considered states
    latest_valid_cur_step = np.inf
    for msg_string in parameters.uav_parameters.observation_msg_strings.values():
        if msg_string in implemented_states_list:
            state_value = getattr(msg,msg_string)
            state_value_latest_cur_step = get_latest_cur_step_for_state(state_value,msg_string,lims_of_cur_steps)
            if state_value_latest_cur_step < latest_valid_cur_step:
                latest_valid_cur_step = state_value_latest_cur_step

    #Get for each relative state the associated discrete state if the discretization of the latest curriculum step is applied
    relative_states = list(lims_of_cur_steps.keys())
    for msg_string in relative_states:
        lims_array = lims_of_cur_steps[msg_string]
        min_value = lims_array[latest_valid_cur_step,0]
        max_value = lims_array[latest_valid_cur_step,1]
        state_value = getattr(msg,msg_string)
        
        # assert parameters.rl_parameters.curriculum_step + 1 < len(parameters.rl_parameters.discretization_steps[msg_string]), "Wrong value for curriculum step. Highest value for curriculum step in parameters file: self.curriculum_step = "+str(len(parameters.rl_parameters.discretization_steps[msg_string])-2)
        assert parameters.rl_parameters.curriculum_step +1 <= len(parameters.rl_parameters.discretization_steps[msg_string]), "Wrong value for curriculum step. Highest value for the curriculum step specified in the parameters file: "+str(len(parameters.rl_parameters.discretization_steps[msg_string])-1)

        #If the latest valid curriculum step is not the latest curriculum step in the curriculum sequence
        if latest_valid_cur_step < len(lims_of_cur_steps[msg_string])-1:
            if "rel_p_" in msg_string:
                #ratio = sigma^2, if lims_of_cur_steps is generated correctly
                ratio = lims_of_cur_steps[msg_string][latest_valid_cur_step + 1][1] / lims_of_cur_steps[msg_string][latest_valid_cur_step][1] 

            elif "rel_v_" in msg_string:
                #ratio = sigma, if lims_of_cur_steps is generated correctly
                ratio = lims_of_cur_steps[msg_string][latest_valid_cur_step + 1][1] / lims_of_cur_steps[msg_string][latest_valid_cur_step][1] 

            elif "rel_a_" in msg_string:
                #ratio = sigma_a
                ratio = parameters.rl_parameters.sigma_a
 
        #If the latest valid curriculum step is the latest curriculum step in the curriculum sequence
        else:
            if "rel_p_" in msg_string:
                ratio = parameters.rl_parameters.beta_value 

            elif "rel_v_" in msg_string:
                ratio = parameters.rl_parameters.beta_value 

            elif "rel_a_" in msg_string:
                ratio = parameters.rl_parameters.beta_value * parameters.rl_parameters.sigma_a

        interval_idx = discretize_relative_state(msg_string,state_value,min_value,max_value,n_r,ratio)
        idx_dict[msg_string] = interval_idx
    return idx_dict,latest_valid_cur_step


def discretize_action(value:float,delta_value:float,max_value:float,eps:float):
    '''
    Function computes the idx of the point that has the same value as "value"
    within an accuracy "eps".
    This point is one of the points that uniformly intersect the interval in the range between 
    -max_value to +max_value with a spacing of delta_value.
    '''
    #Create value range
    values = [-max_value]
    iter_value = -max_value
    while True:
        if max_value - eps/2 <= iter_value <= max_value + eps/2 or iter_value > max_value + eps/2:
            break
        else:
            iter_value += delta_value
            values.append(iter_value)

    #Check if the deviation between the absolute values of the first and last entry in the value range is small enough
    assert abs(abs(values[0])-abs(values[-1])) < eps,"max_value is not an APPROXIMATE integer multiple of delta_value (precision is "+str(eps)+")."

    #Discretize value
    idx = None
    idx_count =0
    loop = 0
    iter_value = -max_value
    while loop <= 1000:
        if abs(iter_value - value) <= eps/2:
            idx = idx_count
            break

        elif iter_value > value:
            print("\033[93mCould not perform the discretization of the action value (value =",value,"). Returning discrete action state =",idx)
            break
            
        else:
            idx_count += 1
            iter_value += delta_value
        loop += 1
    return idx

def get_discrete_action_state_from_ros_msg(msg:Action,action_max_values:dict,action_delta_values:dict):
    '''
    Function computes the discrete state value of an action in the ros msg if that action 
    has been defined in the parameters file.
    Function returns a dictionary. The keys are the action names, the values are the discretized state values.
    '''
    idx_dict = dict()
    for msg_string in action_max_values.keys():
        max_value = action_max_values[msg_string]
        delta_value = action_delta_values[msg_string]
        action_value = getattr(msg,msg_string)
        eps = 1e-05
        idx = discretize_action(action_value,delta_value,max_value,eps)
        idx_dict[msg_string] = idx
    return idx_dict 

def get_discrete_state_from_ros_msg(msg_rel_state:ObservationRelativeState,msg_action_state:Action,lims_of_cur_steps:dict,n_r:int,parameters:Parameters):
    '''
    Function computes the discrete state values (the tuple of grid list idx) from the ros messages of the relative states and the action state values.
    Also the first idx of tuple is added here. It is the most recently added curriculum step to the discretization of which the ros message can be mapped to.
    '''
    rel_state_dict,latest_valid_refinement_step = get_discrete_rel_states_from_ros_msg(msg_rel_state,lims_of_cur_steps,n_r,parameters)
    action_state_dict = get_discrete_action_state_from_ros_msg(msg_action_state,parameters.uav_parameters.action_max_values,parameters.uav_parameters.action_delta_values)
    
    #Build list of indices for the relative states and and action
    idx = list()
    for msg_string in parameters.uav_parameters.observation_msg_strings.values():
        idx.append(rel_state_dict[msg_string])
    for msg_string in parameters.uav_parameters.action_delta_values.keys():
        idx.append(action_state_dict[msg_string])

    #Convert to tuple und insert the latest valid refinement step
    idx = (latest_valid_refinement_step,)+ tuple(idx)
    return idx




