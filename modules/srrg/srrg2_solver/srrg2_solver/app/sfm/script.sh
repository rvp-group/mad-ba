#generate data
rosrun srrg2_solver solver_app_plgo_simulator -zs -gt se3_ba -mt sphere -lsr 8 -lsf 9 -lpp 1 -ml -ma 5 10 10 -mo -o sphere_omni_ba.json
rosrun srrg2_solver solver_app_info_overwrite -c ovt.conf -i sphere_omni_ba.json -o sphere_omni_ba_noise.json
rosrun srrg2_solver solver_app_noise_adder -i sphere_omni_ba_noise.json -o sphere_omni_ba_noise.json
#rosrun srrg2_solver solver_app_graph_initializer -i sphere_omni_ba_noise.json -o sphere_omni_ba_init.json
rosrun srrg2_solver solver_app_graph_sorter -i sphere_omni_ba_noise.json -o sphere_omni_ba_sorted.json

#generate sfm problem
rosrun srrg2_solver plgo2sfm -i sphere_omni_ba_sorted.json -o sphere_omni_ba.sfm
rosrun srrg2_solver sfm2essential -i sphere_omni_ba.sfm -o sphere_omni_ba.ess
rosrun srrg2_solver essential2orientations -ik sphere_omni_ba.sfm -ie sphere_omni_ba.ess -o sphere_omni_ba.sfm
rosrun srrg2_solver essential2translations -ik sphere_omni_ba.sfm -s -100 -ie sphere_omni_ba.ess -o sphere_omni_ba.sfm 
rosrun srrg2_solver sfm2ba -i sphere_omni_ba.sfm -o sphere_omni_ba_est.sfm
