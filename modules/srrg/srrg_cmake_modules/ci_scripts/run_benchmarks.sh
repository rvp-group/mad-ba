#!/bin/bash
#ds this script is intended to be sourced!
#ds it assumes that all dataset resources have been downloaded in advance!

#ds check input parameters
if [ "$#" -lt 2 ]; then
  echo "ERROR: call as $0 REPOSITORY_PATH PROJECT_NAME [WORKSPACE_PATH]"
  exit -1
fi

#ds set parameters
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
echo -e "\e[1;96mbash version: ${BASH_VERSION}\e[0m"
REPOSITORY_PATH="$1"
PROJECT_NAME="$2"
WORKSPACE_PATH="$3"
if [[ -z ${WORKSPACE_PATH} ]]; then
  WORKSPACE_PATH=${WS}
fi

if [[ -z ${SRRG_SCRIPT_PATH} ]]; then
  echo -e "\e[1;91mSRRG_SCRIPT_PATH not set\e[0m"
  exit -1
fi

#ds compose test path and check if existing
EXECUTABLES_PATH="${REPOSITORY_PATH}/${PROJECT_NAME}/benchmarks"
if [ -d "$EXECUTABLES_PATH" ]; then
  #ds parse test binaries from test path
  EXECUTABLES=($(ls ${EXECUTABLES_PATH}))
  echo -e "\e[1;96mfound test files in '${EXECUTABLES_PATH}' (unfiltered): \e[0m"
  echo "${EXECUTABLES[@]}"

  #ds call each binary (skipping all files that do not end in cpp nor start with test)
  for EXECUTABLE in "${EXECUTABLES[@]}"
  do
    #ds binary must start with benchmark keyword and end in .cpp
    EXECUTABLE_PREFIX=${EXECUTABLE:0:9}
    EXECUTABLE_FILE_TYPE=${EXECUTABLE:${#EXECUTABLE}-4:4}
    if [ ${EXECUTABLE_PREFIX} == "benchmark" ] && [ ${EXECUTABLE_FILE_TYPE} == ".cpp" ]; then
      echo -e "\e[1;96m${EXECUTABLE}\e[0m"
      source ${SRRG_SCRIPT_PATH}/run_executable.sh ${PROJECT_NAME} ${EXECUTABLE:0:${#EXECUTABLE}-4} ${WORKSPACE_PATH}

      # echo ""
      # TEXT_EXE="${WS}/devel/${PROJECT_NAME}/lib/${PROJECT_NAME}/${EXECUTABLE:0:${#EXECUTABLE}-4}"
      # echo -e "\e[1;96m${TEXT_EXE}\e[0m"
      # ${TEXT_EXE}
  	fi
  done
else
  echo -e "\e[1;93mbenchmark directory: $EXECUTABLES_PATH is not existing - skipping benchmarks (confirm if running this stage is necessary!)\e[0m"
fi
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
