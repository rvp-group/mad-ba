#!/bin/bash

if [[ $# -ne 5 ]]; then
  echo "usage: $0 <dataset> <config> <gt_topic> <dl_source> <plot_mode {xy,xz,xyz}>"
  return;
  exit -1;
fi

BLACK="\033[30m"
RED="\033[31m"
GREEN="\033[32m"
YELLOW="\033[33m"
BLUE="\033[34m"
MAGENTA="\033[35m"
CYAN="\033[36m"
WHITE="\033[37m"
RESET="\033[0m"
BOLD="\033[1m"

DATASET_NAME="$1"
CONFIG_NAME="$2"
GT_TOPIC="$3"
DL_SOURCE="$4"
PLOT_MODE="$5"
GT_FILE="gt.txt"
TRAJ_FORMAT="tum"

#find config in workspace
CONFIG="$(find $CI_PROJECT_DIR -type f -name ${CONFIG_NAME})"
if [[ ! -f ${CONFIG}  ]]; then
  echo -e "Config "$CYAN $CONFIG_NAME" $RED$BOLD NOT FOUND $RESET in "$YELLOW $CI_PROJECT_DIR
  echo -e $RESET
  return;
  exit -1;
fi
CONFIG=$(realpath "${CONFIG}")

#find dataset in standard dataset directory
if [[ -z "${DATASET_PATH}" ]]; then
  echo -e "$YELLOW DATASET_PATH $RESET variable $RED$BOLD NOT SET $RESET "
  echo -e "Please set it before calling this script"
  echo -e $RESET
  return;
  exit -1;
fi
DATASET="$(find $DATASET_PATH -type f -name ${DATASET_NAME})"
if [[ ! -f ${DATASET}  ]]; then
  echo -e "Dataset "$CYAN $DATASET_NAME" $RED$BOLD NOT FOUND $RESET in "$YELLOW $DATASET_PATH
  echo -e $RESET
  return;
  exit -1;
fi
DATASET=$(realpath "${DATASET}")
cd $(dirname ${DATASET})
echo -e "cd $(dirname ${DATASET})"
echo -e "dataset: $CYAN${DATASET}$RESET"
echo -e "config: $CYAN${CONFIG}$RESET"
echo -e "gt_topic: $CYAN${GT_TOPIC}$RESET"

roscore&
ROSCORE_PID=$!
sleep 2

if [[ -z ${WS} ]]; then
  echo -e "$RED WS not set $RESET"
  exit -1
  return;
fi

DEVEL_PATH="${WS}/devel/"
if [ ${DATASET: -4} == ".bag" ]; then
  EXEC_PATH="${DEVEL_PATH}srrg2_core_ros/lib/srrg2_core_ros/"
  ${EXEC_PATH}extract_gt_from_ros -f ${TRAJ_FORMAT} -i "${DATASET}" -t "${GT_TOPIC}" -o "${GT_FILE}"
else
  EXEC_PATH="${DEVEL_PATH}srrg2_core/lib/srrg2_core/"
  ${EXEC_PATH}extract_gt_from_srrg -f ${TRAJ_FORMAT} -i "${DATASET}" -t "${GT_TOPIC}" -o "${GT_FILE}"
fi
EXEC_PATH="${DEVEL_PATH}srrg2_slam_interfaces/lib/srrg2_slam_interfaces/"
${EXEC_PATH}app_benchmark -dlc ${DL_SOURCE} -c ${CONFIG} -d "${DATASET}" -f ${TRAJ_FORMAT}

kill -9 $ROSCORE_PID

if [[ ! $(pip3 list --format=columns | grep evo)  ]]; then
  echo -e "evo not installed"
  echo -e "run: pip3 install evo --upgrade --no-binary evo"
  exit -1
  return;
fi

RESULTS_APE="results_ape"
PLOT_APE="plot_ape"
evo_ape ${TRAJ_FORMAT} ${GT_FILE} out_${TRAJ_FORMAT}.txt -va --plot_mode ${PLOT_MODE} --save_results ${RESULTS_APE}.zip --no_warnings --save_plot ${PLOT_APE}.png

unzip -p ${RESULTS_APE}.zip stats.json > ${RESULTS_APE}.json

rm ${RESULTS_APE}.zip

RESULTS_RPE="results_rpe"
PLOT_RPE="plot_rpe"
evo_rpe ${TRAJ_FORMAT} ${GT_FILE} out_${TRAJ_FORMAT}.txt -va --plot_mode ${PLOT_MODE} --save_results ${RESULTS_RPE}.zip --no_warnings --save_plot ${PLOT_RPE}.png

unzip -p ${RESULTS_RPE}.zip stats.json > ${RESULTS_RPE}.json

rm ${RESULTS_RPE}.zip
export RESULTS_APE=$(realpath ${RESULTS_APE}.json)
export RESULTS_RPE=$(realpath ${RESULTS_RPE}.json)
export PLOT_APE=$(realpath ${PLOT_APE}.png)
export PLOT_RPE=$(realpath ${PLOT_RPE}.png)
