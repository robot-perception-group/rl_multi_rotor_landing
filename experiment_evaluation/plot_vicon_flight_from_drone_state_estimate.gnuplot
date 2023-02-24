set datafile separator ','
set key opaque box width 2

bag_name = 'vmp_0_4_term_alt_0_4_try_12_20230214_174800.bag'

flyzone_origin_x = -2 #m
flyzone_origin_y = 0 #m

x_max = 1
y_max = 1
min_alt = 1.2
l_mp_half = 0.25

flight_id = 14 #Indexing starts with 0

#Extract the duration of the flight from the data
set terminal unknown
plot "/home/pgoldschmid/src/rl_multi_rotor_landing/experiment_evaluation/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".flight_id."/".bag_name[1:strlen(bag_name)-4]."_state_estimate_pos_vel".".dat" u 1:1# gather basic statistics
t_max=GPVAL_DATA_Y_MAX


set terminal qt size 1000,770
set key outside
set tics font "Times,15"
set key font "Times,15"
set title font "Times,20"
set ylabel font "Times,15"
set xlabel font "Times,15"
set title "Longitudinal Position"
set xlabel "t [s]"
set ylabel "x [m]"
set grid
# set key right top opaque box width 0.5
set print "-"
set datafile separator ","
set multiplot layout 3,1
set ylabel 'r_{mr,x}' 
set yrange [-1.5:1.5]
set xrange [0:t_max]
line_width = 2
set arrow 1 from 0,x_max to t_max,x_max nohead lw line_width lc rgb 'red' 
set arrow 2 from 0,-x_max to t_max,-x_max nohead lw line_width lc rgb 'red'
set title "Longitudinal Position"
set xlabel "t [s]"
set ylabel "x [m]"
plot for [i=flight_id:flight_id] "/home/pgoldschmid/src/rl_multi_rotor_landing/experiment_evaluation/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_state_estimate_pos_vel".".dat" using ($1):($2-flyzone_origin_x) with line lc rgb 'black' title "Vehicle",\
     for [i=flight_id:flight_id] "/home/pgoldschmid/src/rl_multi_rotor_landing/experiment_evaluation/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($2-flyzone_origin_x-l_mp_half) with line notitle,\
     for [i=flight_id:flight_id] "/home/pgoldschmid/src/rl_multi_rotor_landing/experiment_evaluation/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($2-flyzone_origin_x+l_mp_half) with line notitle,\
     for [i=flight_id:flight_id] "/home/pgoldschmid/src/rl_multi_rotor_landing/experiment_evaluation/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using (last_time=$1):($2-flyzone_origin_x+l_mp_half):($2-flyzone_origin_x-l_mp_half) with filledcurves lc "skyblue" fs transparent solid 0.5 title "Platform",\
     1/0 t "Boundary" lt line_width lc rgb 'red' 

set title "Lateral Position"
set xlabel "t [s]"
set ylabel "y [m]"
unset arrow 1
unset arrow 2
set xrange [0:t_max]
set arrow 3 from 0,y_max to t_max,y_max nohead lw line_width lc rgb 'red' 
set arrow 4 from 0,-y_max to t_max,-y_max nohead lw line_width lc rgb 'red'
plot for [i=flight_id:flight_id] "/home/pgoldschmid/src/rl_multi_rotor_landing/experiment_evaluation/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_state_estimate_pos_vel".".dat" using ($1):($3-flyzone_origin_y) with line lc rgb 'black'title "Vehicle",\
     for [i=flight_id:flight_id] "/home/pgoldschmid/src/rl_multi_rotor_landing/experiment_evaluation/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($3-flyzone_origin_y-l_mp_half) with line notitle,\
     for [i=flight_id:flight_id] "/home/pgoldschmid/src/rl_multi_rotor_landing/experiment_evaluation/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($3-flyzone_origin_y+l_mp_half) with line notitle,\
     for [i=flight_id:flight_id] "/home/pgoldschmid/src/rl_multi_rotor_landing/experiment_evaluation/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($3-flyzone_origin_y+l_mp_half):($3-flyzone_origin_y-l_mp_half) with filledcurves lc "skyblue" fs transparent solid 0.5 title "Platform",\
     1/0 t "Boundary" lt line_width lc rgb 'red' 
set xrange [0:t_max]

set title "Vertical Position"
set xlabel "t [s]"
set ylabel "z [m]"
set yrange [0:3]
set xrange [0:t_max]
unset arrow 3
unset arrow 4

plot for [i=flight_id:flight_id] "/home/pgoldschmid/src/rl_multi_rotor_landing/experiment_evaluation/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_drone_pose_enu".".dat" using ($1):($4) with line lc rgb 'black' title "Vehicle",\
     for [i=flight_id:flight_id] "/home/pgoldschmid/src/rl_multi_rotor_landing/experiment_evaluation/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($3-0.3) with line notitle,\
     for [i=flight_id:flight_id] "/home/pgoldschmid/src/rl_multi_rotor_landing/experiment_evaluation/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($3+0.3) with line notitle,\
     for [i=flight_id:flight_id] "/home/pgoldschmid/src/rl_multi_rotor_landing/experiment_evaluation/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($3+0.3):($3-flyzone_origin_y-l_mp_half) with filledcurves lc "skyblue" fs transparent solid 0.5 title "Platform",\
     1/0 t "Boundary" lt line_width lc rgb 'red' 


pause -1 "press any key to close plot"