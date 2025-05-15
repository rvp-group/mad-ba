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
  echo -e "checking ${PROJECT_NAME}"
  RUNNING_PIPELINES=$(curl -s --header "PRIVATE-TOKEN: ${TOKEN}" \
  "https://gitlab.com/api/v4/projects/srrg-software%2F${PROJECT_NAME}/pipelines")
  # echo "${RUNNING_PIPELINES}"
if [[ ${RUNNING_PIPELINES} != "[]" ]]; then
    python_command="
import json
pipelines='${RUNNING_PIPELINES}'
parsed_json=(json.loads(pipelines));
for pipeline in parsed_json:
  if (pipeline['status'] == 'running' or pipeline['status'] == 'pending'):
    print(pipeline['id'])
  "
    FAKE_IDS=$(python -c "${python_command}")
    if [[ ${FAKE_IDS} ]]; then
      readarray -t IDS <<<"$FAKE_IDS"
      for ID in "${IDS[@]}"; do
        echo "$ID"
        RES=$(curl -s --request POST --header "PRIVATE-TOKEN: ${TOKEN}" "https://gitlab.com/api/v4/projects/srrg-software%2F${PROJECT_NAME}/pipelines/${ID}/cancel")
      done
    fi
  fi
done

# JOB_NAME=$(python -c "job='${JOB_NAME}';print(job.replace(job[len('build'):job.find('_ubuntu')], ''))")
