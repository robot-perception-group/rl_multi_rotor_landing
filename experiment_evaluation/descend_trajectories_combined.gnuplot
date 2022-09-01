set tics font "Times,15"
set key font "Times,15"
set title font "Times,20"
set ylabel font "Times,15"
set xlabel font "Times,15"
set datafile separator ','
set terminal qt size 1000,770
set xrange [-5:5]

set grid
set key left bottom opaque box width 0.5


set multiplot
set size 0.5,1
set origin 0,0


#variables function
p_max = 4.5
init_height = 2.4
touchdown_height = 0.4
# p_lims = 0.1111, 0.1678, 0.2621, 0.4096, 0.6400, 1.0000
FILES_failure = system("ls -1 /home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/relative_descend_trajectories/*yaw_0_5.dat")
FILES_success = system("ls -1 /home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/relative_descend_trajectories/*yaw_pi_4_5.dat")

#Draw vertical lines
set arrow from 0.1111*p_max,touchdown_height to 0.1111*p_max,init_height nohead lc rgb 'blue'
set arrow from 0.1678*p_max,touchdown_height to 0.1678*p_max,init_height nohead lc rgb 'blue'
set arrow from 0.2621*p_max,touchdown_height to 0.2621*p_max,init_height nohead lc rgb 'blue'
set arrow from 0.4096*p_max,touchdown_height to 0.4096*p_max,init_height nohead lc rgb 'blue'
set arrow from 0.6400*p_max,touchdown_height to 0.6400*p_max,init_height nohead lc rgb 'blue'

set arrow from -0.1111*p_max,touchdown_height to -0.1111*p_max,init_height nohead lc rgb 'blue'
set arrow from -0.1678*p_max,touchdown_height to -0.1678*p_max,init_height nohead lc rgb 'blue'
set arrow from -0.2621*p_max,touchdown_height to -0.2621*p_max,init_height nohead lc rgb 'blue'
set arrow from -0.4096*p_max,touchdown_height to -0.4096*p_max,init_height nohead lc rgb 'blue'
set arrow from -0.6400*p_max,touchdown_height to -0.6400*p_max,init_height nohead lc rgb 'blue'
# set arrow from -p_max,0 to p_max,-init_height nohead lc rgb 'blue'

set title "Longitudinal Trajectory"
set xlabel "p_{c,x} [m]"
set ylabel "p_{c,z} [m]"

plot for [data in FILES_success] data u 1:3 w l pt 1 lw 2  lc rgb 'green' title  "Yaw = pi/4" 
replot for [data in FILES_failure] data u 1:3 w l pt 1 lw 2 lc rgb 'red' title "Yaw = 0"


show grid


set size 0.5,1
set origin 0.5,0
#variables function
p_max = 4.5
init_height = 2.4
touchdown_height = 0.4
# p_lims = 0.1111, 0.1678, 0.2621, 0.4096, 0.6400, 1.0000
FILES_failure = system("ls -1 /home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/relative_descend_trajectories/*yaw_0_5.dat")
FILES_success = system("ls -1 /home/pgoldschmid/landing_on_moving_platform/other_files/data_analysis/relative_descend_trajectories/*yaw_pi_4_5.dat")

#Draw vertical lines
set arrow from 0.1111*p_max,touchdown_height to 0.1111*p_max,init_height nohead lc rgb 'blue'
set arrow from 0.1678*p_max,touchdown_height to 0.1678*p_max,init_height nohead lc rgb 'blue'
set arrow from 0.2621*p_max,touchdown_height to 0.2621*p_max,init_height nohead lc rgb 'blue'
set arrow from 0.4096*p_max,touchdown_height to 0.4096*p_max,init_height nohead lc rgb 'blue'
set arrow from 0.6400*p_max,touchdown_height to 0.6400*p_max,init_height nohead lc rgb 'blue'

set arrow from -0.1111*p_max,touchdown_height to -0.1111*p_max,init_height nohead lc rgb 'blue'
set arrow from -0.1678*p_max,touchdown_height to -0.1678*p_max,init_height nohead lc rgb 'blue'
set arrow from -0.2621*p_max,touchdown_height to -0.2621*p_max,init_height nohead lc rgb 'blue'
set arrow from -0.4096*p_max,touchdown_height to -0.4096*p_max,init_height nohead lc rgb 'blue'
set arrow from -0.6400*p_max,touchdown_height to -0.6400*p_max,init_height nohead lc rgb 'blue'
# set arrow from -p_max,0 to p_max,-init_height nohead lc rgb 'blue'

set title "Lateral Trajectory"
set xlabel "p_{c,y} [m]"
set ylabel "p_{c,z} [m]"

plot for [data in FILES_success] data u 2:3 w l pt 1 lw 2  lc rgb 'green' title  "Yaw = pi/4" 
replot for [data in FILES_failure] data u 2:3 w l pt 1 lw 2 lc rgb 'red' title "Yaw = 0"


show grid


pause -1 "press any key to close plot"