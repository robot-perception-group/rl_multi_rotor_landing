
# Set the terminal to display the plot
set terminal qt size 1000,770 
set tics font "Times,15"
set key font "Times,15"
set title font "Times,20"
set ylabel font "Times,15"
set xlabel font "Times,15"

set grid
set key left top opaque box width 0.5

# Set the title of the plot
set title "Maximum Attitude Angle (Roll / Pitch)"

# Set the labels for the x and y axes
set ylabel "Attitude Angle [°]"
set xlabel "a_{mp,max} [m/s^2]"

# Define the range for parameter 1
set xrange [0:3]
set yrange [0:30]

# Define the function with two parameters
f(x, k_a) = atan(k_a*x*0.5/9.81)*(180/3.141592)

# Plot the function for different values of parameter 'a'
plot for [k_a=1:10] f(x, k_a) with lines lw 2 title sprintf("k_a=%.1f", k_a*0.5)

# Pause to keep the plot window open
pause -1 "Press Enter to exit"