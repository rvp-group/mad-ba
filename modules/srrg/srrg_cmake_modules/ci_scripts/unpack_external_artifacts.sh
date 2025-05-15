#!/bin/bash
#ds this script is intended to be sourced!

#ds gitlab artifact resources:
#https://docs.gitlab.com/ee/user/project/pipelines/job_artifacts.html
#https://docs.gitlab.com/ee/api/jobs.html
#https://networkinferno.net/token-access-to-files-from-private-repo-in-gitlab

#ds check input parameters
if [[ "$#" < 3 ]]; then
  echo "ERROR: call as $0 PROJECT_NAME BRANCH_NAME JOB_NAME [FALLBACK_BRANCH]"
  exit -1
fi

#ds start
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
echo -e "\e[1;96mbash version: ${BASH_VERSION}\e[0m"
PROJECT_NAME="$1"
BRANCH_NAME="$2"
JOB_NAME="$3"
FALLBACK_BRANCH="$4"
TOKEN="$ARTIFACT_PRIVATE_TOKEN"

echo -e "\e[1;96mPROJECT_NAME: ${PROJECT_NAME}\e[0m"
echo -e "\e[1;96mBRANCH_NAME: ${BRANCH_NAME}\e[0m"
echo -e "\e[1;96mJOB_NAME: ${JOB_NAME}\e[0m"

#ds clone project to catkin source folder (requires SSH key to be properly set up)
cd "/root/workspace/src/"
if [[ "$(catkin_find_pkg $PROJECT_NAME)" ]]; then
  echo -e "\e[1;92mrepo $PROJECT_NAME already cloned\e[0m"
elif [[ $(git ls-remote git@gitlab.com:srrg-software/$1.git) ]]; then
  git clone git@gitlab.com:srrg-software/$1.git
fi


#ds check if selected branch is available
cd "$(catkin_find_pkg $PROJECT_NAME)"
AVAILABLE_BRANCHES=$(git branch -r)
echo -e "\e[1;96mavailable branches in ${PROJECT_NAME}: \e[0m"
echo "$AVAILABLE_BRANCHES"

#ds if the desired branch is available
if [[ $AVAILABLE_BRANCHES == *"origin/${BRANCH_NAME}"* ]]; then
  #ds check out the corresponding branch
  git checkout "$BRANCH_NAME"
elif [[ -n ${FALLBACK_BRANCH} && $AVAILABLE_BRANCHES == *"origin/${FALLBACK_BRANCH}"* ]]; then
  #ds check out the corresponding branch
  BRANCH_NAME="${FALLBACK_BRANCH}"
  git checkout "${BRANCH_NAME}"
else
  #ds otherwise we stay on master (already checked out)
  echo -e "\e[1;93mtarget BRANCH_NAME='${BRANCH_NAME}' not available, staying on 'master'\e[0m"
  BRANCH_NAME="master"
  git checkout "${BRANCH_NAME}"
fi

#ds list last commit (for backtracking)
echo -e "\e[1;96mlatest commit:\e[0m"
git log -q -1

#ds move to workspace
cd "/root/workspace/src/"
echo -e "\e[1;96mcurrent workspace sources: \e[0m"
ls -al

#ds check if the commit hash of the available artifacts corresponds to the last push
#ds if not, we won't be fetching the artifacts and instead perform a rebuild (otherwise we risk build inconsistencies)
echo -e "\e[1;96mchecking status of latest commit for: ${PROJECT_NAME}/${BRANCH_NAME}\e[0m"
LATEST_COMMIT_URL="https://gitlab.com/api/v4/projects/srrg-software%2F${PROJECT_NAME}/repository/commits/${BRANCH_NAME}"
COMMIT_STATUS=$(curl --header "PRIVATE-TOKEN: $TOKEN" "$LATEST_COMMIT_URL")
if [[ $COMMIT_STATUS != *"\"status\":\"success\""* ]]; then
  echo -e "\e[1;93mcommit not successful - skipping artifact import\e[0m"
  echo -e "$COMMIT_STATUS"
#  if [ -z "$CATKIN_WHITELIST" ]; then
#    CATKIN_WHITELIST="$PROJECT_NAME ${PROJECT_NAME}_ros"
#  else
#    CATKIN_WHITELIST="${CATKIN_WHITELIST} $PROJECT_NAME ${PROJECT_NAME}_ros"
#  fi
#  catkin config --whitelist $CATKIN_WHITELIST
#  export CATKIN_WHITELIST
  return #ds statement has no effect if script is not sourced
  exit -1 #ds escape in any case (skipped when sourcing, otherwise fatal)
fi
echo -e "\e[1;96mstatus:success - importing artifacts\e[0m"

#ds assemble project artifact URL
ARTIFACT_DOWNLOAD_URL="https://gitlab.com/api/v4/projects/srrg-software%2F${PROJECT_NAME}/jobs/artifacts/${BRANCH_NAME}/download?job=${JOB_NAME}"

#ds download artifacts into the catkin workspace folder using the gitlab api
cd "/root/workspace/"
wget --no-verbose --header "PRIVATE-TOKEN: $TOKEN" "$ARTIFACT_DOWNLOAD_URL" --output-document "artifacts.zip"

#ds unzip artifacts into corresponding folders and remove file containers
unzip artifacts.zip
rm artifacts.zip
tar xzf artifacts/build.tar.gz
tar xzf artifacts/devel.tar.gz
rm -rf artifacts

#ds blacklist the loaded project in catkin to disable rebuilding in any circumstances
#ds we have to extend the previous blacklist since the command is not appending
if [ -z "$CATKIN_BLACKLIST" ]; then
  CATKIN_BLACKLIST="$PROJECT_NAME ${PROJECT_NAME}_ros"
else
  CATKIN_BLACKLIST="${CATKIN_BLACKLIST} $PROJECT_NAME ${PROJECT_NAME}_ros"
fi
catkin config --blacklist $CATKIN_BLACKLIST
export CATKIN_BLACKLIST
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
