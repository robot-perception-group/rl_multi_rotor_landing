set tics font "Times,15"
set key font "Times,15"
set title font "Times,20"
set ylabel font "Times,15"
set xlabel font "Times,15"
set datafile separator ','
set terminal qt size 1000,770
# set nokey
set key left opaque box width 2
set termoption enhanced


set xlabel "Episode Number "
set ylabel "Accumulated Reward per Episode"
set grid

set multiplot
set size 1,0.5
set origin 0, 0.5

#File paths
file_path_1 = "/home/pgoldschmid/rl_multi_rotor_landing/experiment_evaluation/rewards/vmp_1_6_sim_1_training_results_training_q_learning_1.csv"
file_path_2 = "/home/pgoldschmid/rl_multi_rotor_landing/experiment_evaluation/rewards/vmp_1_6_sim_1_training_results_training_q_learning_2.csv"
file_path_3 = "/home/pgoldschmid/rl_multi_rotor_landing/experiment_evaluation/rewards/vmp_1_6_sim_1_training_results_training_q_learning_3.csv"
file_path_4 = "/home/pgoldschmid/rl_multi_rotor_landing/experiment_evaluation/rewards/vmp_1_6_sim_1_training_results_training_q_learning_4.csv"
file_path_5 = "/home/pgoldschmid/rl_multi_rotor_landing/experiment_evaluation/rewards/vmp_1_6_sim_1_training_results_training_q_learning_5.csv"

#Variables
num_eps_1 = 1726
num_eps_2 = 97
num_eps_3 = 96
num_eps_4 = 96
num_eps_5 = 98

#Functions
samples5(x) = $0 > 4 ? 5 : ($0+1)
avg5(x) = (shift5(x), (back1+back2+back3+back4+back5)/samples5($0))
shift5(x) = (back5 = back4, back4 = back3, back3 = back2, back2 = back1, back1 = x)

samples10(x) = $0 > 4 ? 10 : ($0+1)
avg10(x) = (shift10(x), (back1+back2+back3+back4+back5+back6+back7+back8+back9+back10)/samples10($0))
shift10(x) = (back10 = back9, back9 = back8, back8 = back7, back7 = back6, back6 = back5,back5 = back4, back4 = back3, back3 = back2, back2 = back1, back1 = x)

#Init
init5(x) = (back1 = back2 = back3 = back4 = back5 = sum = 0)
init10(x) = (back1 = back2 = back3 = back4 = back5 = back6 = back7 = back8 = back9 = back10 = sum = 0)



#Draw vertical lines
min_reward = -155
max_reward = 155
set arrow 1 from num_eps_1,min_reward to num_eps_1,max_reward nohead  lw 2 lc rgb 'red'
set arrow 2 from num_eps_2+num_eps_1,min_reward to num_eps_2+num_eps_1,max_reward nohead lw 2 lc rgb 'red'
set arrow 3 from num_eps_3+num_eps_2+num_eps_1,min_reward to num_eps_3+num_eps_2+num_eps_1,max_reward nohead lw 2lc rgb 'red'
set arrow 4 from num_eps_4+num_eps_3+num_eps_2+num_eps_1,min_reward to num_eps_4+num_eps_3+num_eps_2+num_eps_1,max_reward nohead lw 2lc rgb 'red'
set arrow 5 from num_eps_5+num_eps_4+num_eps_3+num_eps_2+num_eps_1,min_reward to num_eps_5+num_eps_4+num_eps_3+num_eps_2+num_eps_1,max_reward nohead lw 2 lc rgb 'red'


#plot
set xrange[0:2150]
set yrange [min_reward:max_reward]
set title "RPM 1.6/1"
lw_variance = 5
lw_average = 3
plot sum = init10(0), \
     file_path_1 using 2:3 title 'Accumulated Reward per Episode' w l lw lw_variance+1 lc rgb '#AA32CD32', \
     '' using 2:(avg10($3)) title 'Moving Average (n=10)'  w l lw lw_average lc rgb '#00000000',\
     sum = init10(0), \
     file_path_2 using (column(2)+num_eps_1):3 notitle w l lw lw_variance lc rgb '#AA32CD32', \
     file_path_2 using (column(2)+num_eps_1):(avg10($3)) notitle w l lw lw_average lc rgb "#00000000",\
     sum = init10(0), \
     file_path_3 using (column(2)+num_eps_2+num_eps_1):3 notitle w l lw lw_variance lc rgb '#AA32CD32', \
     file_path_3 using (column(2)+num_eps_2+num_eps_1):(avg10($3)) notitle w l lw lw_average lc rgb "#00000000",\
     sum = init10(0), \
     file_path_4 using (column(2)+num_eps_3+num_eps_2+num_eps_1):3 notitle w l lw lw_variance lc rgb '#AA32CD32', \
     file_path_4 using (column(2)+num_eps_3+num_eps_2+num_eps_1):(avg10($3)) notitle w l lw lw_average lc rgb "#00000000",\
     sum = init10(0), \
     file_path_5 using (column(2)+num_eps_4+num_eps_3+num_eps_2+num_eps_1):3 notitle  w l lw lw_variance lc rgb '#AA32CD32', \
     file_path_5 using (column(2)+num_eps_4+num_eps_3+num_eps_2+num_eps_1):(avg10($3)) notitle w l lw lw_average lc rgb "#00000000"



#File paths
file_path_new_1 = '/home/pgoldschmid/rl_multi_rotor_landing/experiment_evaluation/rewards/vmp_0_4_real_sim_3_training_results_training_q_learning_1.csv'
file_path_new_2 = '/home/pgoldschmid/rl_multi_rotor_landing/experiment_evaluation/rewards/vmp_0_4_real_sim_3_training_results_training_q_learning_2.csv'
file_path_new_3 = '/home/pgoldschmid/rl_multi_rotor_landing/experiment_evaluation/rewards/vmp_0_4_real_sim_3_training_results_training_q_learning_3.csv'
file_path_new_4 = '/home/pgoldschmid/rl_multi_rotor_landing/experiment_evaluation/rewards/vmp_0_4_real_sim_3_training_results_training_q_learning_4.csv'

#Variables

num_eps_1 = 2054
num_eps_2 = 97
num_eps_3 = 96
num_eps_4 = 96
unset arrow

set size 1,0.5
set origin 0, 0

set arrow 6 from num_eps_1,min_reward to num_eps_1,max_reward nohead  lw 2 lc rgb 'red'
set arrow 7 from num_eps_2+num_eps_1,min_reward to num_eps_2+num_eps_1,max_reward nohead lw 2 lc rgb 'red'
set arrow 8 from num_eps_3+num_eps_2+num_eps_1,min_reward to num_eps_3+num_eps_2+num_eps_1,max_reward nohead lw 2lc rgb 'red'
set arrow 9 from num_eps_4+num_eps_3+num_eps_2+num_eps_1,min_reward to num_eps_4+num_eps_3+num_eps_2+num_eps_1,max_reward nohead lw 2lc rgb 'red'


set xrange[0:2400]
set  yrange [min_reward:max_reward]
set title "RPM 0.4/3"
plot sum = init10(0), \
     file_path_new_1 using 2:3 title 'Accumulated Reward per Episode' w l lw lw_variance+1 lc rgb '#AA32CD32',\
     '' using 2:(avg10($3)) title 'Moving Average (n=10)'  w l lw lw_average lc rgb '#00000000',\
     sum = init10(0), \
     file_path_new_2 using (column(2)+num_eps_1):3 notitle w l lw lw_variance lc rgb '#AA32CD32', \
     file_path_new_2 using (column(2)+num_eps_1):(avg10($3)) notitle w l lw lw_average lc rgb "#00000000",\
     sum = init10(0), \
     file_path_new_3 using (column(2)+num_eps_2+num_eps_1):3 notitle w l lw lw_variance lc rgb '#AA32CD32', \
     file_path_new_3 using (column(2)+num_eps_2+num_eps_1):(avg10($3)) notitle w l lw lw_average lc rgb "#00000000",\
     sum = init10(0), \
     file_path_new_4 using (column(2)+num_eps_3+num_eps_2+num_eps_1):3 notitle w l lw lw_variance lc rgb '#AA32CD32', \
     file_path_new_4 using (column(2)+num_eps_3+num_eps_2+num_eps_1):(avg10($3)) notitle w l lw lw_average lc rgb "#00000000"
    
pause -1 "press any key to close plot"
