from socketserver import ThreadingUnixDatagramServer
import numpy as np
from os.path import join, isfile
from os import listdir, walk
import pandas as pd
from copy import deepcopy
from datetime import datetime, timedelta


sim_ids = ['sim_1','sim_2','sim_3','sim_4']
tb_string = 'training_q_learning'
training_id = 'vmp_0_4_real'
experiment_path = '/home/pgoldschmid/Desktop/exp_paper_20230109/'
training_lengths_dict = dict()
training_nums_of_episodes_dict = dict()

#Iterate through the different simulations
for sim_id in sim_ids:
    #Determine the number of folders that contain training data
    training_folders = [x[0] for x in walk(join(experiment_path,training_id,sim_id,"training_results",""))]

    #Select only the paths that contain the tb_string
    training_folder_dirs = [x for x in training_folders if tb_string in x and "config" not in x]

    
    #Determine the path to the log files of the init position
    file_paths = []
    for dir_path in training_folder_dirs:
        files = [f for f in listdir(dir_path) if isfile(join(dir_path, f))]
        for file in files:
            if "init_data.csv" in file:
                file_paths.append(join(dir_path,file))

    #Sort file_paths alphabetically to get order right
    file_paths = sorted(file_paths)  
    # print("file_paths =",file_paths)  

    #Iterate through all the init files and import them as numpy array
    datas = list()
    for file_path in file_paths:
        df = pd.read_csv(file_path,skiprows = 0,delimiter = ',') #Skip first episode which is there due to reset during training init
        df.columns = range(df.shape[1]) #Replace column names with integer values
        datas.append( deepcopy(df.to_numpy()))

    total_time = timedelta(0)
    total_number_of_episodes = 0
    training_lengths = []
    training_nums_of_episodes = []

    #Iterate through each training
    print("=====================")
    print("Sim ID:",sim_id)
    for i in range(len(file_paths)):
        #Get the training time data for the first and last training of that simulation
        training_start_first_str = datas[i][0,0]
        training_start_last_str = datas[i][-1,0]

        #Convert to time variables
        training_start_first = datetime.strptime(training_start_first_str,'%Y%m%d_%H%M%S')
        training_start_last = datetime.strptime(training_start_last_str,'%Y%m%d_%H%M%S')
        training_length = training_start_last-training_start_first

        #Create list of training durations in minutes
        training_lengths.append(np.rint(training_length.total_seconds()/60))

        #Create list with numbers of episodes
        training_num_of_episodes = len(datas[i])+1
        training_nums_of_episodes.append(training_num_of_episodes)
        
        print(i+1,"Training dur.", training_length,"[",np.rint(training_length.total_seconds()/60),"]")
        print(i+1,"Training no. ",training_num_of_episodes)
        print("- - - - - - - - - - ")

        total_number_of_episodes += training_num_of_episodes
        total_time += training_length
    print("------------------- ")
    print("Total time",total_time,"[",np.rint(total_time.total_seconds()/60),"]")
    print("Total number of episodes",total_number_of_episodes)
    
    #Add the results to the dict 
    training_nums_of_episodes_dict[sim_id] = training_nums_of_episodes
    training_lengths_dict[sim_id] = training_lengths



#Get the required info for each training
print(" ")
print("============================")
print("Mean and Std computed for the ",len(sim_ids),'agents for the different curriculum steps')
print('-----------------------')
for i in range(len(file_paths)):
    training_lengths_stats = [training_lengths_dict [key][i] for key in sorted(training_lengths_dict.keys()) ]
    training_nums_of_episodes_stats = [training_nums_of_episodes_dict [key][i] for key in sorted(training_nums_of_episodes_dict.keys()) ]
    print(i, "Length: Mean +- Std",np.rint(np.mean(training_lengths_stats)),'+-',np.rint(np.std(training_lengths_stats)))
    print(i, "No. ep: Mean +- Std", np.rint(np.mean(training_nums_of_episodes_stats)),'+-',np.rint(np.std(training_nums_of_episodes_stats)))
    print('-------------------')
total_training_time_list = []
total_num_of_eps_list = []
#Determine the total training duration
for sim_id in sim_ids:
    training_length_tmp = training_lengths_dict[sim_id]
    num_of_eps_tmp = training_nums_of_episodes_dict[sim_id]
    
    total_training_time_tmp = np.sum(training_length_tmp)
    total_num_of_eps_tmp = np.sum(num_of_eps_tmp)
    
    total_training_time_list.append(total_training_time_tmp)
    total_num_of_eps_list.append(total_num_of_eps_tmp)


#Determine the total number of episodes
print("Total duration: Mean: ",np.rint(np.mean(total_training_time_list)),"| Std.",np.rint(np.std(total_training_time_list)))
print("Tot. num. of e: Mean: ",np.rint(np.mean(total_num_of_eps_list)),"| Std.",np.rint(np.std(total_num_of_eps_list)))

print("\033[93mThis script was not used to determine the results of the training duration / number of episodes. These were based on the recorded tensorboard log files.")
