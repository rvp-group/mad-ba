#!/bin/bash
echo "Script start"
source /opt/ros/noetic/setup.bash
source ../../../devel/setup.bash
roscd structure_refinement/config/
rosrun structure_refinement main_app -c maths_institute.config
roscd structure_refinement/scripts
python3 compareClouds_maths_institute.py >> ../output/eval/"$(date +%F-%T)".txt
echo "Script end"
