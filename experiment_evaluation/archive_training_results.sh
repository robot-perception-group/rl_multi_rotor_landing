#This script creates a folder structure ato store training results and test results for up to six simulations

#Read parameters
exp_id=$1
target_root=$2
sim_root=$3


#Create folder structure
for sim in sim_1 sim_2 sim_3 sim_4 sim_5 sim_6 test_results; do
    mkdir -p $target_root/$exp_id/$sim
done
echo "Created folder structure."

#Copy training results
for sim in sim_1 sim_2 sim_3 sim_4 sim_5 sim_6; do
    echo "Begin copying training results of simulation $sim..."
    cp -r $sim_root/$sim/test_landing/rl_multi_rotor_landing_sim/src/training_q_learning/training_results $target_root/$exp_id/$sim
    echo "Done"
done
echo "Success!"
