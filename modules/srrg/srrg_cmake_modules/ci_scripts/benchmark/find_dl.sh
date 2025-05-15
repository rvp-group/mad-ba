#!/bin/bash

cd ${WS}/src/
git clone git@gitlab.com:srrg-software/srrg2_executor.git
cd srrg2_executor
git checkout $CI_BUILD_REF_NAME
cd ${WS}
catkin build srrg_cmake_modules --no-deps
catkin build srrg2_executor --no-deps
${WS}/devel/srrg2_executor/lib/srrg2_executor/auto_dl_finder
export DLC=$(realpath ${WS}/dl.conf)
