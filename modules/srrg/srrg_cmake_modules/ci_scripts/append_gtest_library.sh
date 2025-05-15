
#!/bin/bash

##### tg This script is added in order to run tests on multiple package in the same git repo 
##### libgtest.so is appended to the artifacts, this is necessary because of catkin build
##### have individual build folder for each package

#ds check input parameters
if [ "$#" -ne 2 ]; then
  echo "ERROR: call as $0 PROJECT_ROOT_PATH PROJECT_NAME"
  exit -1
fi

#ds parameters
PROJECT_ROOT_PATH="$1"
PROJECT_NAME="$2"
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
echo -e "\e[1;96mbash version: ${BASH_VERSION}\e[0m"
cd "/root/workspace/"
ls -al

#ds determine gtest library location in the build folder
GTEST_LIBRARY_PATH=$(find "build/${PROJECT_NAME}" -name "libgtest.so")
if [ -z ${GTEST_LIBRARY_PATH} ]; then
  GTEST_LIBRARY_PATH=$(find "build/${PROJECT_NAME}" -name "libgtestd.so")
fi
echo -e "\e[1;96mGTEST_LIBRARY_PATH='${GTEST_LIBRARY_PATH}'\e[0m"
if [ ! -z "$GTEST_LIBRARY_PATH" ]; then
  cd ${PROJECT_ROOT_PATH}/artifacts/
  tar xzf build.tar.gz
  rm build.tar.gz
  GTEST_LIBRARY_PATH_PREVIOUS=$(find "build/" -name "libgtest.so")
  if [ -z ${GTEST_LIBRARY_PATH_PREVIOUS} ]; then
    GTEST_LIBRARY_PATH_PREVIOUS=$(find "build/" -name "libgtestd.so")
  fi
  echo -e "\e[1;96mGTEST_LIBRARY_PATH_PREVIOUS='${GTEST_LIBRARY_PATH_PREVIOUS}'\e[0m"
  cd "/root/workspace/"
  tar czf ${PROJECT_ROOT_PATH}/artifacts/build.tar.gz "$GTEST_LIBRARY_PATH" "$GTEST_LIBRARY_PATH_PREVIOUS"
fi
#ds log available artifacts
ls -al "${PROJECT_ROOT_PATH}/artifacts/"
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
