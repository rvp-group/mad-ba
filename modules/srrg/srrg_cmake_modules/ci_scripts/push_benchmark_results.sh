#!/bin/bash
#ds this script is intended to be sourced!

#ds check input parameters
if [ "$#" -ne 4 ]; then
  echo "ERROR: call as $0 RESULT_REPOSITORY_PATH PROJECT_NAME DATASET_NAME COMMIT_MESSAGE"
  exit -1
fi

#ds commit hash needs to be set
if [ -z "$CI_COMMIT_SHA" ]; then
  echo "ERROR: CI commit SHA not set"
  exit -1
fi

#ds start
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
echo -e "\e[1;96mbash version: ${BASH_VERSION}\e[0m"
RESULT_REPOSITORY_PATH="$1"
PROJECT_NAME="$2"
DATASET_NAME="$3"
COMMIT_MESSAGE="$4"

#ds update reference commit in the provided result repository
cd "$RESULT_REPOSITORY_PATH"
git config --global user.email "benchamin@srrg.com"
git config --global user.name "benchamin"

echo -e "\e[1;96mgit stash\e[0m"
if [[ $(git stash) == *"No stash entries"* ]]; then
  echo -e "\e[1;96mnothing to commit, bye\e[0m"
  return 0
fi

git checkout devel
git stash pop
git add "results"
ls -al

#ds if the result content has changed (does not have to be the case)
if [[ $(git status) != *"nothing to commit, working tree clean"* ]]; then

  #ds make sure we're fresh before pushing (if there is a discontinuity in the results folder we abort)
  echo -e "\e[1;96mgit pull\e[0m"
  git pull

  #ds update reference commit link in README for current dataset
  sed -i "s|\[${DATASET_NAME}\]\: https://gitlab.com/srrg-software/${PROJECT_NAME}/commit/.*|\[${DATASET_NAME}\]\: https://gitlab.com/srrg-software/${PROJECT_NAME}/commit/${CI_COMMIT_SHA}|g" "readme.md"
  echo -e "\e[1;96mupdated entry: [${DATASET_NAME}] to commit ${CI_COMMIT_SHA} \e[0m"
  tail "readme.md"

  #ds push to remote without triggering it's ci
  echo -e "\e[1;96mgit commit -am [automatic][benchmark][${PROJECT_NAME}] ${COMMIT_MESSAGE} [skip ci]\e[0m"
  git commit -am "[automatic][benchmark][${PROJECT_NAME}] ${COMMIT_MESSAGE} [skip ci]"

  #ds push - can be ignored if there were no changes after pulling
  echo -e "\e[1;96mgit push origin devel\e[0m"
  git push origin devel
else
  #ds ignore push
  echo -e "\e[1;96mbenchmark results have not changed - ignoring commit and push\e[0m"
fi
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
