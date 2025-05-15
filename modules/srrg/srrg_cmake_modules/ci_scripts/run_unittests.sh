#!/bin/bash

#ds check input parameters
if [ "$#" -lt 2 ]; then
  echo "ERROR: call as $0 REPOSITORY_PATH PROJECT_NAME [WORKSPACE_PATH]"
  exit -1
fi

#ds set parameters
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
echo -e "\e[1;96mbash version: ${BASH_VERSION}\e[0m"
REPOSITORY_PATH="$1"  #TODO remove this useless variable
PROJECT_NAME="$2"
WORKSPACE_PATH="$3"
if [[ -z ${WORKSPACE_PATH} ]]; then
  WORKSPACE_PATH=${WS}
fi

if [[ -z ${SRRG_SCRIPT_PATH} ]]; then
  echo -e "\e[1;91mSRRG_SCRIPT_PATH not set\e[0m"
  return -1
  exit -1
fi

EXECUTABLE_FOLDER="${WORKSPACE_PATH}/devel/${PROJECT_NAME}/lib/${PROJECT_NAME}/"
if [ ! -d "$EXECUTABLE_FOLDER" ]; then
  echo -e "\e[1;93mexecutable directory: $EXECUTABLE_FOLDER is not existing"
  return -1
  exit -1
fi

TEST_BINARIES=($(ls ${EXECUTABLE_FOLDER}))
echo -e "\e[1;96mfound test files in '${EXECUTABLE_FOLDER}' (unfiltered): \e[0m"
echo "${TEST_BINARIES[@]}"

for TEST_BINARY in "${TEST_BINARIES[@]}"
do
  #ds binary must start with test keyword and end in .cpp
  TEST_BINARY_PREFIX=${TEST_BINARY:0:4}
  if [ ${TEST_BINARY_PREFIX} == "test" ]; then
    source ${SRRG_SCRIPT_PATH}/run_executable.sh ${PROJECT_NAME} ${TEST_BINARY} ${WORKSPACE_PATH}
    # echo ""
    # TEXT_EXE="${WORKSPACE_PATH}/devel/${PROJECT_NAME}/lib/${PROJECT_NAME}/${TEST_BINARY:0:${#TEST_BINARY}-4}"
    # echo -e "\e[1;96m${TEXT_EXE}\e[0m"
    # ${TEXT_EXE}
  fi
done
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
