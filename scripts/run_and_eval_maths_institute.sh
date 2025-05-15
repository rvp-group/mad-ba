#!/bin/bash
echo "Script start"
source /opt/ros/noetic/setup.bash
source ../../../devel/setup.bash
roscd mad_ba/config/
rosrun mad_ba main_app -c maths_institute.config
roscd mad_ba/scripts
python3 compareClouds_maths_institute.py >> ../output/eval/"$(date +%F-%T)".txt
echo "Script end"
