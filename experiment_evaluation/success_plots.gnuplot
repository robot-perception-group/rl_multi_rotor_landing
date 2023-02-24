## Input 
#Define file to load success rates from
file_name_rpm_0_8 = "success_rates_sim/success_rates_rpm_0_8.dat"
file_name_rpm_1_2 = "success_rates_sim/success_rates_rpm_1_2.dat"
file_name_rpm_1_6 = "success_rates_sim/success_rates_rpm_1_6.dat"
file_name_rpm_baseline = "success_rates_sim/success_rates_baseline.dat"

#Define number of agents trained for one parameter setting
N_ag = 4

#Define number of different experiments scenarios that were evaluated
N_eval_exps = 6 

color_train_case_rpm_0_8 = "red"
color_train_case_rpm_1_2 = "web-green"
color_train_case_rpm_1_6 = "blue"

## Calc
#Define terminal 
set terminal qt size 950,462

#Define fonts
set tics font "Times,15"
set key font "Times,15"
set title font "Times,20"
set ylabel font "Times,15"
set xlabel font "Times,15"

#Define plot
set title "Simulation Success Rates without Noise"
set ylabel "Success Rate [%]"
set xrange [0:7]
# set key below inside
set key outside vertical right


set key opaque box width 2
# set nokey
set xtics ("Static" 1, "RPM 0.4" 2, "RPM 0.8" 3, "RPM 1.2" 4, "RPM 1.6" 5, "Eight shape" 6)
set ytics  0,2,100
set grid ytics
array pointtypes = [1,2,10,12]
var_pointsize = 2


#Compute statistics for errobars
stats file_name_rpm_0_8 skip 0
max_col_0_8 = STATS_columns
array col_mean_0_8[max_col_0_8]
array col_std_0_8[max_col_0_8]
do for [i=1:max_col_0_8]{
    stats file_name_rpm_0_8 using i nooutput
    col_mean_0_8[i] = STATS_mean
    col_std_0_8[i] = STATS_stddev
}
stats file_name_rpm_1_2 skip 0
max_col_1_2 = STATS_columns
array col_mean_1_2[max_col_1_2]
array col_std_1_2[max_col_1_2]
do for [i=1:max_col_1_2]{
    stats file_name_rpm_1_2 using i nooutput
    col_mean_1_2[i] = STATS_mean
    col_std_1_2[i] = STATS_stddev
}
stats file_name_rpm_1_6 skip 0
max_col_1_6 = STATS_columns
array col_mean_1_6[max_col_1_6]
array col_std_1_6[max_col_1_6]
do for [i=1:max_col_1_6]{
    stats file_name_rpm_1_6 using i nooutput
    col_mean_1_6[i] = STATS_mean
    col_std_1_6[i] = STATS_stddev
}

#Create plot
set yrange [68:100]

plot for [i in "1 2 3 6"] file_name_rpm_0_8 using 2*i-1: (col_mean_0_8[2*i]) : (col_std_0_8[2*i]) with errorbars linecolor "grey" linewidth 2 notitle,\
     for [j=0:N_ag-1] for [i in "1"] file_name_rpm_0_8 every ::j::j using 2*i-1:2*i with points pointtype pointtypes[j+1] pointsize var_pointsize linewidth 2 linecolor "red" title "RPM 0.8/".(j+1),\
     for [j=0:N_ag-1] for [i in "2 3 6"] file_name_rpm_0_8 every ::j::j using 2*i-1:2*i with points pointtype pointtypes[j+1] pointsize var_pointsize linewidth 2 linecolor "red" notitle,\
     for [i in "1 2 3 4 6"] file_name_rpm_1_2 using 2*i-1: (col_mean_1_2[2*i]) : (col_std_1_2[2*i]) with errorbars linecolor "grey" linewidth 2 notitle,\
     for [j=0:N_ag-1] for [i in "1"] file_name_rpm_1_2 every ::j::j using 2*i-1:2*i with points pointtype pointtypes[j+1] pointsize var_pointsize linewidth 2 linecolor "blue" title "RPM 1.2/".(j+1),\
     for [j=0:N_ag-1] for [i in "2 3 4 6"] file_name_rpm_1_2 every ::j::j using 2*i-1:2*i with points pointtype pointtypes[j+1] pointsize var_pointsize linewidth 2 linecolor "blue" notitle,\
     for [i in "1 2 3 4 5 6"] file_name_rpm_1_6 using 2*i-1: (col_mean_1_6[2*i]) : (col_std_1_6[2*i]) with errorbars linecolor "grey" linewidth 2 notitle,\
     for [j=0:N_ag-1] for [i in "1"] file_name_rpm_1_6 every ::j::j using 2*i-1:2*i with points pointtype pointtypes[j+1] pointsize var_pointsize linewidth 2 linecolor "web-green" title "RPM 1.6/".(j+1),\
     for [j=0:N_ag-1] for [i in "2 3 4 5 6"] file_name_rpm_1_6 every ::j::j using 2*i-1:2*i with points pointtype pointtypes[j+1] pointsize var_pointsize linewidth 2 linecolor "web-green" notitle,\
     file_name_rpm_baseline using 1:2 with line linewidth 2 linecolor "black" title "Baseline",\
     file_name_rpm_baseline using 3:4 with line linewidth 2 linecolor "black" notitle 
pause -1