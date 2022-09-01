import numpy as np
from training_q_learning.utils import load_csv_values_as_nparray_for_state
from training_q_learning.parameters import Parameters




#General variables
implemented_states_list = ["rel_p_x","rel_p_y","rel_p_z","rel_v_x","rel_v_y","rel_v_z","rel_yaw","rel_a_x","rel_a_y","rel_a_z"]


def initialize_grid_list(grid,init_value):
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




def convert_value_to_grid_idx(value,min_value,max_value,n_r):
    '''Function computes the idx (starting with zero) of the n_r intervals that intersect the range from min_value to max_value
    in which the value 'value' resides.'''

    assert n_r>0,"n_r must be greater than zero (currently n_r = "+str(n_r)+")"
    assert value <= max_value,"value must be less or equal to maximum value (value = "+str(value)+" and maximum value = "+str(max_value)+")"
    assert value >= min_value,"value must be greater or equal to minimum value (value ="+str(value)+" and minimum value ="+str(min_value)+")"

    value_range = np.linspace(min_value,max_value,n_r +1)#Plus one because n_r specifies the number of intervals

    idx = np.argmax(value_range >= value) -1

    if value == min_value:
        idx = 0
    if value == max_value:
        idx = n_r -1
    return idx 
    
def convert_value_to_grid_idx_dynamic_model(state_name:str,value:float,min_value:float,max_value:float,n_r:int,ratio:float):
    '''
    Function converts a value to the associated grid idx. 
    The grid is assumed to be constructed as follows.
        - For values greater zero:
            - The parameter ratio defines the proportion by which the absolute  max_value / min_value is reduced in order one obtain the positive / negative goal value (the value of 
    the border of the goal interval). 
            -The range betweeen the positive / negative goal value and the max_value / min_value is then intersected with additional boundaries separating different intervals. 
            The number of additional boundaries is computed in such a way that the total number of intervals between min_value and max_value is n_r. The range between the positive / negative goal value and the
    max_value / min_value is intersected in intervals of equal size.
    '''
    parameters = Parameters()
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

    #Determine the idx of the interval in which the value resides
    idx = np.argmax(value_range >= value) -1

    #Consider boundary cases
    if value == min_value:
        idx = 0
    if value == max_value:
        idx = n_r -1

    #If a negative idx should be determined, print the case for which it happend
    if idx < 0:
        print("value=",value)
        print("min_value=",min_value)
        print("max_value=",max_value)    
    return idx

def convert_value_to_linear_global_grid_idx(state_name:str,value:float,min_value:float,max_value:float,n_r:int,chi:float):
    '''
    Function converts a value to the associated idx of the grid interval in which it resides. The grid is assumed to be constructed as follows.
    The parameter chi defines the proportion by which the absolute max_value / min_value is reduced in order to obtain the positive / negative goal value (the value of 
    the border of the goal goal intervals). The range between the positive / negative goal value and max_value / min_value is then intersected with additional boundaries separating intervals. 
    The number of additional boundaries is computed in such a way that the number of intervals n_r between min_value and max_value is n_r. The range between the positive / negative goal value and the
    and the max_value / min_value is intersected in intervals of equal size.
    '''
    assert max_value >= min_value,"The max. value needs to be greater or equal than the min. value. Current min. value: "+str(min_value)+", current max. value "+str(max_value)
    assert n_r>0,"n_r must be greater than zero (currently n_r = "+str(n_r)+")"
    assert value <= max_value,"value must be less or equal to maximum value (value = "+str(value)+" and maximum value = "+str(max_value)+")"
    assert value >= min_value,"value must be greater or equal to minimum value (value ="+str(value)+" and minimum value ="+str(min_value)+")"
    assert state_name in implemented_states_list,"State "+state_name+" not in list of implemented states. Implemented states: "+", ".join(implemented_states_list)
    assert int(n_r+1) % 2 == 0,"n_r needs to be an odd number. Even numbers are not implemented yet. "+str(int((n_r+1)/2) % 2)
    #Get normalized shrink increment of 
    delta_chi = 1-chi
    #Determine the positive border of the goal grid idx
    pos_border_goal_value = max_value-delta_chi

    #Validate data
    assert pos_border_goal_value < max_value,"The normalized goal state value ("+str(pos_border_goal_value)+") is not lower than the max value ("+str(max_value)+") for state "+state_name

    #Intersect the range from goal grid idx border to max value in such a way that the total sum of intervals between min value and max value is n_r
    pos_half_value_range = np.linspace(pos_border_goal_value,max_value,int((n_r+1)/2))

    #Determine the positive border of the goal grid idx
    neg_border_goal_value = min_value+delta_chi

    #Intersect the range from min value to negative goal grid idx border in such a way that the total sum of intervals between min value and max value is n_r
    neg_half_value_range = np.linspace(min_value,neg_border_goal_value,int((n_r+1)/2))
  
    #Build the range of values that define the intervals
    value_range = np.concatenate([neg_half_value_range,pos_half_value_range])

    #Determine the idx of the interval in which the value resides
    idx = np.argmax(value_range >= value) -1

    #Handling of boundary values
    if value == min_value:
        idx = 0
    if value == max_value:
        idx = n_r -1

    #If a negative idx occurs communicate under which conditions
    if idx < 0:
        print("value=",value)
        print("min_value=",min_value)
        print("max_value=",max_value)
    
    return idx

    
def add_to_dict_refinement_min_max_values_for_state(state:str,data_dict:dict):
    '''Function reads the discretization steps defined for state "state" in the appropriate csv file files in the config folder. 
    It uses these values to construct a dictionary that contains the positive and negative limit values for each state.
    The resulting dictionary is returned. '''

    parameters = Parameters()
    minmax_array = np.array([]).reshape(0,2)
    #Check if a list of discretization steps exist for the specified state
    if state in implemented_states_list:
        state_name = state[-3:]
        discretization_steps = load_csv_values_as_nparray_for_state(state_name,'training_q_learning')

        #asert validity
        assert sorted(list(discretization_steps)) == list(discretization_steps),'The discretization steps defined for state "'+ state + '" are not sorted in ascending order.'

        #build the return array
        i = 0
        for step in discretization_steps:            
            if i >= (len(discretization_steps)-1) - (parameters.rl_parameters.curriculum_step):
                minmax_array = np.insert(minmax_array,0,[-step,step],axis = 0)
            i += 1
        data_dict[state] = minmax_array
    return minmax_array

def get_refinement_step_of_state_value(value:float,state:str,data_dict:dict):
    '''
    Function determines the idx of the latest refinement step whose limit values define a range in which the current state value resides.
    If the state value is equal to a limit value of that discretization step, the returned refinement step is the refinement step that was added later.
    The counting of the refinement steps starts with zero.
    If the value does not fall into the bounds of any refinement step, the initial refinement step 0 is returned and a warning is printed.
    '''
    step_idx = 0
    minmax_array = data_dict[state]

    #Validity check
    assert sorted(list(minmax_array[:,0])) == list(minmax_array[:,0]),'The min and max values are not sorted in ascending order for increasing refinement steps'
    #Count refinement steps that include the value in the allowed value range
    for i in range(len(minmax_array)):
        if minmax_array[(i,0)] <= value <= minmax_array[(i,1)]:
            step_idx += 1
    #Subtract 1 to consider indexing starting with zero
    step_idx = step_idx - 1 
    if step_idx < 0:
        #If the value is outside any range provided by the minmax_array
        step_idx = 0
        print('\033[93m'+'Detected value ('+str(value)+') outside specified refinement steps for state', state,'. Will return refinement step idx 0.' + '\033[0m')
    return step_idx
    
    

def get_rel_state_grid_list_idx_from_ros_msg(msg,refinement_steps_dict:dict,n_r:int,parameters):
    ''' Function maps the state values received in the ros msg to the indices of the discrete states specified by the limit values in refinement_steps_dict in which the current values reside. 
        These indices comply with the following conditions.
            - For each continouos state value, the function determines the index of the discrete state (there are n_r discrete states in total for each continouos state) that is associated with the latest refinement step that can be applied to all states.
            - The latest refinement step is the refinement step for which all state values in "msg" are framed by the limit values associdated with that refinement step.
        The Function returns a dictionary. The keys are the used message field names, the values are the indices belonging to that message.
    '''    
    idx_dict = {}
    #Determine the latest refinement step of all considered states
    latest_valid_refinement_step = np.inf
    for msg_string in parameters.uav_parameters.observation_msg_strings.values():
        if msg_string in implemented_states_list:
            state_value = getattr(msg,msg_string)
            state_value_latest_refinement_step = get_refinement_step_of_state_value(state_value,msg_string,refinement_steps_dict)
            if state_value_latest_refinement_step < latest_valid_refinement_step:
                latest_valid_refinement_step = state_value_latest_refinement_step

    #Get the interval index for the value of each state in the ros message for the latest refinement step
    relative_states = list(refinement_steps_dict.keys())
    for msg_string in relative_states:
        minmax_array = refinement_steps_dict[msg_string]
        min_value = minmax_array[latest_valid_refinement_step,0]
        max_value = minmax_array[latest_valid_refinement_step,1]
        state_value = getattr(msg,msg_string)
        #Select the specified discretization scheme
        if parameters.rl_parameters.discretization_mode == 'interval_fixed_step_length':
            interval_idx = convert_value_to_grid_idx(state_value,min_value,max_value,n_r)
        elif parameters.rl_parameters.discretization_mode == 'global_fixed_step_length':
            interval_idx = convert_value_to_linear_global_grid_idx(msg_string,state_value,min_value,max_value,n_r,parameters.rl_parameters.chi)
        elif parameters.rl_parameters.discretization_mode == 'dynamic_model':
            #The length of refinement_steps_dict corresponds to the number of ref steps specified in the config files
            if latest_valid_refinement_step < len(refinement_steps_dict[msg_string])-1:
                ratio = refinement_steps_dict[msg_string][latest_valid_refinement_step + 1][1] / refinement_steps_dict[msg_string][latest_valid_refinement_step][1]   
            else:
                #The refinement_steps_dict only contains the boundary values of the discretization steps, so the goal state of the highest ref step
                #can be computed as follows
                latest_max_value = refinement_steps_dict[msg_string][latest_valid_refinement_step][1]
                pos_goal_state = (1/parameters.rl_parameters.n_r) * latest_max_value
                ratio = pos_goal_state/latest_max_value
            interval_idx = convert_value_to_grid_idx_dynamic_model(msg_string,state_value,min_value,max_value,n_r,ratio)
        idx_dict[msg_string] = interval_idx
    return idx_dict,latest_valid_refinement_step



def get_action_value_grid_list_idx(value,delta_value,max_value,eps):
    '''
    Function computes the idx of the point that has the same value as "value"
    within an accuracy "eps".
    This point is one of the points that uniformly intersect the interval in the range between 
    -max_value to +max_value with a spacing of delta_value.
    '''
    #Create range
    values = [-max_value]
    iter_value = -max_value
    loop = 0
    while loop <= np.inf:
        if max_value - eps/2 <= iter_value <= max_value + eps/2:
            break
        elif iter_value > max_value:
            break
        else:
            iter_value += delta_value
            values.append(iter_value)
        loop += 1

    assert abs(abs(values[0])-abs(values[-1])) < eps,"max_value is not an APPROXIMATE  integer multiple of delta_value (precision is "+str(eps)+")."

    idx = None
    idx_count =0
    loop = 0
    iter_value = -max_value
    while loop <= 1000:
        if abs(iter_value - value) <= eps/2:
            idx = idx_count
            break
        elif iter_value > value:
            break
        else:
            idx_count += 1
            iter_value += delta_value
        loop += 1
    return idx

def get_action_state_grid_idx(msg,action_max_values,action_delta_values):
    '''
    Function computes the discrete state value of an action in msg, if that action 
    has been defined in the parameters file.
    Function returns a dictionary. The keys are the action names, the values are the discrete state values.
    '''
    parameters = Parameters()
    idx_dict = dict()
    for msg_string in action_max_values.keys():
        max_value = action_max_values[msg_string]
        delta_value = action_delta_values[msg_string]
        value = getattr(msg,msg_string)
        eps = 1e-05
        #######
        # This is for real action values not the setpoint ones
        # idx = get_action_grid_list_idx(value,delta_value,max_value,eps)
        #######
        #######
        #This is for the setpoint action values not the real ones ones
        idx = get_action_value_grid_list_idx(value,delta_value,max_value,eps)
        #######
        idx_dict[msg_string] = idx
    return idx_dict 

def get_state_grid_idx_from_ros_msg(msg_rel_state,msg_action_state,refinement_steps_dict:dict,n_r:int,parameters):
    '''
    Function computes the discrete state values (the tuple of grid list idx) from the ros messages for the relative states and the action state values.
    Also the first idx of tuple is added here. It is the latest refinement step for which all state values reside within the respective min max intervals that are associated with that refinement step.
    '''
    rel_state_dict,latest_valid_refinement_step = get_rel_state_grid_list_idx_from_ros_msg(msg_rel_state,refinement_steps_dict,n_r,parameters)
    action_state_dict = get_action_state_grid_idx(msg_action_state,parameters.uav_parameters.action_max_values,parameters.uav_parameters.action_delta_values)
    
    idx = list()
    for msg_string in parameters.uav_parameters.observation_msg_strings.values():
        idx.append(rel_state_dict[msg_string])
    for msg_string in parameters.uav_parameters.action_delta_values.keys():
        idx.append(action_state_dict[msg_string])

    #Convert to tuple und insert the latest valid refinement step
    idx = (latest_valid_refinement_step,)+ tuple(idx)
    return idx


def get_lowest_discretization_step_for_multiresolution_as_dict(refinement_step:int,state_name_long:str,n_r:int,refinement_steps_dict:dict):
    '''
    Function returns the latest refinement step if the discretization files in the config folder
    are interpreted as the values that define the limits of the value ranges associated with the different refinement steps.
    '''
    assert isinstance(refinement_step,int),"refinement_step needs to be an integer"
    assert n_r > 1,"n_r needs to be an odd integer value greater than 1"
    assert isinstance(n_r,int),"n_r needs to be of type integer"
    assert state_name_long in implemented_states_list,"The specified state does not exist"
    state_name = state_name_long[-3:]

    limit_value = refinement_steps_dict[state_name_long][refinement_step][1]
        
    value_range = np.linspace(-limit_value,limit_value,n_r+1)

    #if n_r is an odd number
    if n_r % 2 != 0:    
        idx = int((n_r+1)/2)
    else:
        idx = int((n_r+2)/2)
    lowest_discretization_step = value_range[idx]

    return lowest_discretization_step    

