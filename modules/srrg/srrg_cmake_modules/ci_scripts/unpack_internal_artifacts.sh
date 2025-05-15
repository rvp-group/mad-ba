#!/bin/bash

#ds gitlab artifact resources:
#https://docs.gitlab.com/ee/user/project/pipelines/job_artifacts.html
#https://docs.gitlab.com/ee/api/jobs.html
#https://networkinferno.net/token-access-to-files-from-private-repo-in-gitlab

#ds check input parameters
if [[ "$#" < 2 ]]; then
  echo "ERROR: call as $0 PROJECT_ROOT_PATH ARTIFACTS_JOB_NAME [FALLBACK_BRANCH]"
  exit -1
fi

#ds start
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
echo -e "\e[1;96mbash version: ${BASH_VERSION}\e[0m"
cd "/root/workspace/"
PROJECT_ROOT_PATH="$1"
ARTIFACTS_JOB_NAME="$2"
FALLBACK_BRANCH="$3"

#ds restore build artifacts from previous corresponding stage of this project
tar xzf "${PROJECT_ROOT_PATH}/artifacts/build.tar.gz"
tar xzf "${PROJECT_ROOT_PATH}/artifacts/devel.tar.gz"

#srrg import external artifacts TODO pass this values as arguments
# JOB_NAME="${CI_JOB_NAME}"
# if [[ $CI_JOB_NAME == *test* ]]; then
#   JOB_NAME="$(echo "${CI_JOB_NAME}" | sed 's/test/build/')"
# elif [[ $CI_JOB_NAME == *benchmark* ]]; then
#   JOB_NAME="$(echo "${CI_JOB_NAME}" | sed 's/benchmark/build/')"
# fi
#
# JOB_NAME=$(python -c "job='${JOB_NAME}';print(job.replace(job[len('build'):job.find('_ubuntu')], ''))")
source ${SRRG_SCRIPT_PATH}/unpack_all_external_artifacts.sh $CI_PROJECT_NAME $CI_BUILD_REF_NAME $ARTIFACTS_JOB_NAME $FALLBACK_BRANCH

ls -al
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
