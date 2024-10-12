#!/bin/bash
rosrun structure_refinement main_app -c config/VBR/campus.config
rosrun structure_refinement main_app -c config/VBR/ciampino.config
rosrun structure_refinement main_app -c config/VBR/colosseo.config
rosrun structure_refinement main_app -c config/VBR/diag.config
rosrun structure_refinement main_app -c config/VBR/pincio.config
rosrun structure_refinement main_app -c config/VBR/spagna.config

campus.config  ciampino.config  colosseo.config  diag.config  pincio.config  spagna.config
