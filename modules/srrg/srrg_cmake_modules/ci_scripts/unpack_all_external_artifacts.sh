#!/bin/bash
#ds this script is intended to be sourced!

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

cd /root/workspace/

function clone_repo() {
  cd "/root/workspace/src/"
  if [[ "$(catkin_find_pkg $1)" ]]; then
    echo -e "\e[1;92mrepo $1 already cloned\e[0m"
    return 0;
  fi
  if [[ $(git ls-remote git@gitlab.com:srrg-software/$1.git) ]]; then
    git clone git@gitlab.com:srrg-software/$1.git
    create_tree $1
    create_tree $1_ros
  else
    echo -e "\e[1;93mrepo $1 does not exists in srrg-software group\e[0m"
  fi
}
function create_tree() {
  echo -e "Deps for repo \e[1;96m$1\e[0m"
  if [[ ! "$(catkin_find_pkg $1)" ]]; then
    echo -e "\e[1;93mcatkin package $1 not found\e[0m"
    return 0;
  fi
  PACK_DIR="$(catkin_find_pkg $1)"
  echo -e "found catkin package $1 in ${PACK_DIR}"
  cd "${PACK_DIR}"
  SRRG_DEPS="$(catkin list --this --deps | awk '/build_depend/,/run_depend/{print $2}' | xargs -0 echo | awk '/srrg2/{print $0}'\
  |  tac)"
  echo "${SRRG_DEPS[@]}"

  for dep in $SRRG_DEPS; do
    clone_repo "$dep"
  done
}
create_tree $PROJECT_NAME

cd /root/workspace/

cd "$(catkin_find_pkg ${PROJECT_NAME})"

SRRG_RDEPS="$(catkin list --this --rdeps | awk '/build_depend/,/run_depend/{print $2}' | xargs -0 echo | awk '/srrg2/{print $0}' |\
tac)"
echo "Recursive dependencies: "
echo "${SRRG_RDEPS[@]}"
echo ""
for LIB in $SRRG_RDEPS; do
  echo -e "\e[1;96mDownloading $LIB artifacts\e[0m";
  source ${SRRG_SCRIPT_PATH}/unpack_external_artifacts.sh "$LIB" "$BRANCH_NAME" "$JOB_NAME" "$FALLBACK_BRANCH"
done
