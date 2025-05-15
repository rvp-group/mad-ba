#!/bin/bash

INPUT_FILE=${1}
if [[ ! -f ${INPUT_FILE} ]]; then
  echo "no repos file as first"
  exit -1
fi
declare -a PROJECT_NAMES
PROJECT_NAMES=()

while IFS= read -r line; do
  PROJECT_NAMES+=(${line})
done < ${INPUT_FILE}

TOKEN=${2}
if [[ -z {PROJECT_NAME} ]]; then
  echo "no token given as second argument"
  exit -1
fi

# echo "${PROJECT_NAMES[@]}"
for PROJECT_NAME in "${PROJECT_NAMES[@]}"; do
  echo -e "starting ${PROJECT_NAME} CI pipeline"
  ./start_pipeline.sh ${PROJECT_NAME} ${TOKEN}
done
