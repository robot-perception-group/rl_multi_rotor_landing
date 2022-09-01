clear all;

%% Input

n_r = 3;

w_p_x = 1;

n_theta = 3;
k = 15;
n_ax = 3;

v_mp = 1.6;
r_mp = 2;

theta_0 = deg2rad(0);
g = 9.81;


%% Calc

ax_mp = v_mp^2/r_mp;
ax_drone = n_ax*ax_mp;

omega_mp = v_mp/r_mp;
f_mp = omega_mp/(2*pi);

theta_limit = atan2(ax_drone,g);

% def zero function
func =  @(x) n_theta*(atan2(w_p_x*x^2,(2*g))-theta_0)-theta_limit;
f_agent_accuracy = fzero(func,5);

delta_theta = atan2(w_p_x*f_agent_accuracy^2,(2*g));

f_agent_maneuver = 4*k*n_theta*f_mp;
f_agent = max([f_agent_accuracy,f_agent_maneuver]);
ax_drone_calc = tan(theta_limit)*g;

%% Vis

display("theta_limit = "+rad2deg(theta_limit)+"deg")
display("delta_theta = "+rad2deg(delta_theta)+"deg")

display("f_agent = "+f_agent + "hz")
display("ax_drone = "+num2str(ax_drone_calc)+ "m/(s^2)")
display("ax_mp = "+num2str(ax_mp)+ "m/(s^2)")
display("ax_drone multiple = "+num2str(ax_drone) + "m/(s^2)")

% end
if f_agent_accuracy > f_agent_maneuver
   display("NOTE: Accuracy is dominating f_agent") 
    
else
   display("NOTE: Maneuver is dominating f_agent") 

end

%% Functions
function y = log_base(base,value)

y = log10(value)/log10(base);

end
