#!/bin/bash

#ds check input parameters
if [ "$#" -ne 2 ]; then
  echo "ERROR: call as $0 PROJECT_DIRECTORY PROJECT_NAME"
  exit -1
fi

#ds set parameters
PROJECT_DIRECTORY="$1"
PROJECT_NAME="$2"
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
echo -e "\e[1;96mbash version: ${BASH_VERSION}\e[0m"

#ds create catkin workspace and link this repository for build with catkin
mkdir -p /root/workspace/src
ln -s "$PROJECT_DIRECTORY" "/root/workspace/src/${PROJECT_NAME}"

#ds setup test data path (routed through source directory for local compatibility)
mkdir -p /root/source/srrg && mkdir -p /root/source/srrg2
ln -s "$PROJECT_DIRECTORY" "/root/source/srrg2/${PROJECT_NAME}"

catkin config -w /root/workspace --init --isolate-devel
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
