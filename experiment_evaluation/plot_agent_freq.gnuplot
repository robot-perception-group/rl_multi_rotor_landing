
# Set the terminal to display the plot
set terminal qt size 1000,770 
set tics font "Times,15"
set key font "Times,15"
set title font "Times,20"
set ylabel font "Times,15"
set xlabel font "Times,15"

set grid
set key right top opaque box width 0.5

# Set the title of the plot
set title "Agent Frequency"

# Set the labels for the x and y axes
set ylabel "Agent frequency f_{ag} [hz]"
set xlabel "Angular Velocity of Moving Platform [1/s^2]"

# Define the range for parameter 1
set xrange [0:3]
set yrange [0:30]

# Define the function with two parameters
n_theta = 3
f(x, k_man) = 2*n_theta*k_man*x/3.141592

# Plot the function for different values of parameter 'a'
plot for [k_man=3:30:3] f(x, k_man) with lines lw 2 title sprintf(" k_{man}=%.d", k_man)

# Pause to keep the plot window open
pause -1 "Press Enter to exit"