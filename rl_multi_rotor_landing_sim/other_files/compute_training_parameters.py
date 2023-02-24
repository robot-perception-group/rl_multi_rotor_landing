"""
This script can be used to compute the values of the most important parameters specified in the parameter file that define the RL training.
"""

import numpy as np

## Input
# Define factors
k_a = 3 #[-]
k_man = 15 #[-]
n_theta = 3 #[-]

#Define parameters of platform 
v_mp = 1.6 #[m/s]
r_mp = 2 #[m]
l_mp = 1 #[m]

#Define parameters of fly zone
x_max = 1 #[m]

#Define parameters of reward function
w_p = -100 #[-]
w_v = -10 #[-]
w_theta = -1.55 #[-]
w_dur = -6.0 #[-] 

#Other parameters
g = 9.81 #[m/s^2]
sigma = 0.8 #[-]
sigma_acc = 0.416 #[-]

## Calc
#Determine values
omega_mp = v_mp/r_mp 
f_mp = omega_mp/(2*np.pi)
a_mpmax = np.round_(v_mp**2/r_mp,decimals=5)

#Determine hyperparameters
theta_max = np.arctan2(k_a*a_mpmax,g)
f_ag = 2*n_theta*k_man*omega_mp/np.pi
delta_theta = theta_max/n_theta

## Compute boundaries
#Compute min and max time in the worst case scenario 
t_0 = np.sqrt(2*(x_max/a_mpmax))
n_cs = int(np.ceil(0.5*(np.log(l_mp/(2*x_max)) / np.log(sigma)) - 1))

#Compute set of timestamps T
T_list = []
for i in range(n_cs+1):
    T_list.append(t_0*sigma**i)
T = np.array(T_list)

#Compute the limit values of the discrete positions
p_lims = 0.5*a_mpmax*T**2 / x_max

#Commpute the limit values of the discrete velocity
v_max = a_mpmax*t_0
v_lims = a_mpmax*T / v_max

#Compute the limit values of the discrete acceleration
a_lims = np.array([1.0]*(n_cs + 1))

#Compute maximum achievable reward per timestep
delta_t = (1/f_ag)
r_pmax_list = abs(w_p)*v_lims*delta_t
r_vmax_list = abs(w_v)*a_lims*delta_t
r_thetamax_list = abs(w_theta)*(delta_theta/theta_max)*v_lims
r_durmax_list = w_dur*v_lims*delta_t

r_max_list = r_pmax_list + r_vmax_list + r_thetamax_list + r_durmax_list
r_max_ratio_list = []
for i in range(1,n_cs+1):
    r_max_ratio_list.append(r_max_list[i]/r_max_list[i-1])
    
## Vis
print("------------------------")
print("General parameters")
print("x_max =",x_max,"m")
print("l_mp =",l_mp,"m")
print("k_a =",k_a)
print("k_man =",k_man)
print("n_theta =",n_theta)
print("sigma =",sigma)
print("sigma_acc =",sigma_acc)
print("v_mp =",v_mp,"m/s")
print("r_mp =",r_mp,"m")
print("a_mpmax =",a_mpmax,"m/s^2")
print("omega_mp =",omega_mp,"rad/s")
print("f_mp =",f_mp,"hz")
print("n_cs =",n_cs)


print("------------------------")
print("UAV parameters")
print("theta_max [deg] = {:.5f}".format(theta_max*(180/np.pi)))
print("delta_theta [deg] = {:.5f}".format(delta_theta*(180/np.pi)))
print("NOTE: Use full values for max. pitch angle and pitch angle increment. The devision 'theta_max/delta_theta' needs to yield n_theta =",str(n_theta),"with a possible deviation of 1e-05.")
print("---")
print("normalization value rel_p_x =",x_max)
print("normalization value rel_v_x =",np.round_(v_max,decimals=5))
print("normalization value rel_a_x =",a_mpmax)

print("------------------------")
print("RL parameters")
print("max. reward ratio (scale_modification_value)")
print(r_max_ratio_list)
print("Limit values for discretized relative position")
print(list(np.round_(p_lims,decimals=8)))
print("---")
print("Limit values for discretized relative velocity")
print(list(np.round_(v_lims,decimals=8)))
print("---")
print("Limit values for discretized relative acceleration")
print(list(np.round_(a_lims,decimals=2)))
print("---")
print("f_ag [hz] = {:.2f}".format(f_ag))

