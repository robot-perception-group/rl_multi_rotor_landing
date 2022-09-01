clear all;clc;close all;



%% Input

p_min = 0.5;
f_agent =22.9183;   %hz


grad_start_rel_p = -100;
grad_start_rel_v = -10;
action_weight = 1.55;


n_r = 3;
reward_duration = -6*(1/f_agent);

p_max = 4.5; % m
a_max = 1.28; % m/s
max_pitch = deg2rad(21.377);
delta_pitch = deg2rad(7.1257);

dt = 0.001;

t_max = sqrt(2*p_max/a_max);
contraction_factor = 0.8;

% timesteps = [linspace(0.913,t_max,m+1)] ;

%% Calc



%time for plotting
t = 0:dt:t_max;

%Decisive for goal state
t_p_min = sqrt(2*p_min/a_max);
% t_v_min = v_min / a_max;

% t_min = min([t_p_min,t_v_min]);
t_min = t_p_min;

timesteps = [t_max];
i = 1;
%build t_vec
while t_min < min(timesteps)
%     t_new = (timesteps(end))*contraction_factor;
%     timesteps = [timesteps;t_new];
    timesteps = [contraction_factor*timesteps(1);timesteps];
    
     i = i+1;
end

timesteps_pos = timesteps;
timesteps_vel = timesteps;

timesteps_pos(timesteps < t_p_min) = t_p_min;
% timesteps_vel(timesteps < t_v_min) = t_v_min;

% timesteps_pos = [contraction_factor*min(timesteps_pos);timesteps_pos];
% timesteps_vel = [contraction_factor*min(timesteps_vel);timesteps_vel];
% timesteps = [contraction_factor*min(timesteps);timesteps];
%% Vis
v_max = a_max*t_max;
s = (1/2)*a_max*t.^2;
v = a_max*t;

figure('name','determine t discretization for goal state')
plot(t,s)
hold on
grid on
yyaxis right
ylabel("v_x [m/s]")
plot(t,v)
xlabel("t [s]")
yyaxis left
ylabel("p_x [m]")
plot(timesteps,(1/2)*a_max*timesteps_pos.^2,'o')

yyaxis right
plot(timesteps,a_max*timesteps_vel,'o')
title("Discretization")



% Determine the values

   p_steps = ((1/2)*a_max*timesteps_pos.^2)/p_max;
   v_steps = (a_max*timesteps_vel)/v_max;
   a_steps = ones(size(timesteps));
    
p_steps_string = num2str(p_steps',' %.4f,');
v_steps_string = num2str(v_steps',' %.4f,');
a_steps_string = num2str(a_steps',' %.4f,');
p_steps_string = p_steps_string(1:end-1);
v_steps_string = v_steps_string(1:end-1);
a_steps_string = a_steps_string(1:end-1);

p_steps_string
v_steps_string
a_steps_string



%compute the max reward in one timestep
reward_rel_pos_max = abs(grad_start_rel_p)*(1/f_agent)*v_steps;
reward_rel_vel_max = abs(grad_start_rel_v)*(1/f_agent)*a_steps;
reward_rel_action_max = (action_weight/max_pitch)*delta_pitch*v_steps;
 reward_duration_max = reward_duration* v_steps;

max_reward_per_step = flip(reward_rel_pos_max + reward_rel_vel_max + reward_rel_action_max+reward_duration_max);
% max_reward_per_step = flip(reward_rel_pos_max + reward_rel_vel_max + reward_rel_action_max);

reward_ratio = [];
for i = 1:1:max(size(max_reward_per_step))-1
    reward_ratio = [reward_ratio,max_reward_per_step(i+1)/max_reward_per_step(i)]; 
end

reward_ratio_string = num2str(reward_ratio,' %.4f,');
reward_ratio_string = reward_ratio_string(1:end-1)


function a = a(t)
% Function computes the value of a at a given


end
