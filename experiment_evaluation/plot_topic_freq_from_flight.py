import rosbag
import matplotlib.pyplot as plt
import os
import pandas as pd


file_base_path = 'rosbag_converts'
bag_name = "vmp_0_4_cascaded_pi_try_12_20240122_121706.bag"
flight_id = 0
number_window_samples = 150 #frequency determined over the time window specified by the number of samples


##Calc
id_name = bag_name[:-4]
file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),file_base_path,bag_name[:-4],'Flights','flight_'+str(flight_id),id_name+"_state_estimate_pos_vel"+'.dat')

def decompose_vector(input_vector, subvector_length):
    # Calculate the number of subvectors
    num_subvectors = len(input_vector) // subvector_length

    # Initialize the list to store subvectors
    subvectors = []

    # Iterate over the input vector and create subvectors
    for i in range(num_subvectors):
        start_index = i * subvector_length
        end_index = (i + 1) * subvector_length
        subvector = input_vector[start_index:end_index]
        subvectors.append(subvector)

    # Handle the last subvector if the lengths don't sum up to the input vector length
    if len(input_vector) % subvector_length != 0:
        last_subvector = input_vector[num_subvectors * subvector_length:]
        subvectors.append(last_subvector)

    return subvectors

def calcualte_windowed_freqs_over_time(timestamps, number_window_samples):
    windowed_freqs = []
    if timestamps.size:
        windowed_freqs = list()
        windowed_timestamps = decompose_vector(timestamps,number_window_samples)
        for windowed_timestamp in windowed_timestamps:
            windowed_freq = len(windowed_timestamp)/(windowed_timestamp[-1]-windowed_timestamp[0])
            windowed_freqs.extend([windowed_freq]*len(windowed_timestamp))
        
    return timestamps,windowed_freqs
    
def create_plot(timestamps,windowed_freqs,number_window_samples,bag_name,flight_id):
    if timestamps.size:
        fig = plt.figure(figsize=(12,10))
        plt.plot(timestamps, windowed_freqs)
        plt.xlabel('Time [s]')
        plt.ylabel('Frequency [hz]')
        plt.title(f"Frequency of State Estimate\nROSbag file: {bag_name}\nFlight ID: {flight_id}\nSamples for Windowing: "+str(number_window_samples))
        # plt.title(f'Frequency of Topic: {topic_name}\nROSbag file: {bag_file_path}\nNumber window samples:{number_window_samples}')
        plt.grid()
        plt.ylim([0,150])
        plt.show()
    else:
        print(f"Error.")









def plot_topic_frequency(file_path,number_window_samples,bag_name,flight_id):
    column_names = ['timestamps', 'px', 'py', 'pz','vx','vy','vz']
    data = pd.read_csv(file_path,header=None,sep=',',names=column_names)
    timestamps = data["timestamps"].to_numpy()
    
    print("Calculating windowed frequencies...")
    _,windowed_frequencies = calcualte_windowed_freqs_over_time(timestamps, number_window_samples)
    print("Done calculating windowed frequencies...")
    print("Creating plot")
    create_plot(timestamps,windowed_frequencies,number_window_samples,bag_name,flight_id)
    print("Done")


if __name__ == "__main__":


    plot_topic_frequency(file_path,number_window_samples,bag_name,flight_id)
