set tics font "Times,15"
set key font "Times,15"
set title font "Times,20"
set ylabel font "Times,15"
set xlabel font "Times,15"
set datafile separator ','
set terminal qt size 1000,770
set key outside


set title "Longitudinal Position"
set xlabel "t [s]"
set ylabel "x [m]"
set grid
# set key right top opaque box width 0.5
set print "-"
set datafile separator ","
set multiplot layout 3,1
set ylabel 'r_{mr,x}' 
# set y2label 'r_{mr,y}'
# set y2tics mirror 
# set nokey

set key opaque box width 2


bag_name = 'stmoving_good_parameters_slow_20220824_204346.bag'

flyzone_origin_x = -2 #m
flyzone_origin_y = 0 #m

x_max = 1
y_max = 1
l_mp_half = 0.25
t_max = 16.5

#Flight to be shorted: flight 6
flight_id = 6


set xrange [0:t_max]
set yrange [-1.5:1.5]
line_width = 2
set arrow 1 from 0,y_max to t_max,y_max nohead lw line_width lc rgb 'red' 
# set arrow from 0,l_mp_half to t_max,l_mp_half nohead 
set arrow 2 from 0,-y_max to t_max,-y_max nohead lw line_width lc rgb 'red'
# set arrow from 0,-l_mp_half to t_max,-l_mp_half nohead 
set title "Longitudinal Position"
set xlabel "t [s]"
set ylabel "x [m]"
plot for [i=flight_id:flight_id] "/home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_state_estimate_pos_vel".".dat" using ($1):($2-flyzone_origin_x) with line lc rgb 'black' title "Vehicle",\
     for [i=flight_id:flight_id] "/home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($2-flyzone_origin_x-l_mp_half) with line notitle,\
     for [i=flight_id:flight_id] "/home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($2-flyzone_origin_x+l_mp_half) with line notitle,\
     for [i=flight_id:flight_id] "/home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($2-flyzone_origin_x+l_mp_half):($2-flyzone_origin_x-l_mp_half) with filledcurves lc "skyblue" fs transparent solid 0.5 title "Platform",\
     1/0 t "Boundary" lt line_width lc rgb 'red' 
set title "Lateral Position"
set xlabel "t [s]"
set ylabel "y [m]"
plot for [i=flight_id:flight_id] "/home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_state_estimate_pos_vel".".dat" using ($1):($3-flyzone_origin_y) with line lc rgb 'black'title "Vehicle",\
     for [i=flight_id:flight_id] "/home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($3-flyzone_origin_y-l_mp_half) with line notitle,\
     for [i=flight_id:flight_id] "/home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($3-flyzone_origin_y+l_mp_half) with line notitle,\
     for [i=flight_id:flight_id] "/home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($3-flyzone_origin_y+l_mp_half):($3-flyzone_origin_y-l_mp_half) with filledcurves lc "skyblue" fs transparent solid 0.5 title "Platform",\
     1/0 t "Boundary" lt line_width lc rgb 'red' 
set title "Vertical Position"
set xlabel "t [s]"
set ylabel "z [m]"
set yrange [0:3]
unset arrow 2
set arrow 3 from 0,y_max to t_max,y_max nohead lw line_width lc rgb 'red' 
plot for [i=flight_id:flight_id] "/home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_state_estimate_pos_vel".".dat" using ($1):(-$4) with line lc rgb 'black' title "Vehicle",\
     for [i=flight_id:flight_id] "/home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($3-0.3) with line notitle,\
     for [i=flight_id:flight_id] "/home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($3+0.3) with line notitle,\
     for [i=flight_id:flight_id] "/home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($3+0.3):($3-flyzone_origin_y-l_mp_half) with filledcurves lc "skyblue" fs transparent solid 0.5 title "Platform",\
     1/0 t "Boundary" lt line_width lc rgb 'red' 
# replot 
# replot for [i=flight_id:flight_id] "/home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($2-flyzone_origin_x) with line title "Flight {".i."}"
# replot for [i=flight_id:flight_id] "/home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/rosbag_converts/".bag_name[1:strlen(bag_name)-4]."/Flights/flight_".i."/".bag_name[1:strlen(bag_name)-4]."_mp_pose".".dat" using ($1):($3-flyzone_origin_y) with line title "Flight {".i."}"

pause -1 "press any key to close plot"