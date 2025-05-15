#!/usr/bin/env bash

PROJECT_NAME=${1}

if [[ -z {PROJECT_NAME} ]]; then
  echo "set a repo name"
  exit -1
fi

TOKEN=${2}
if [[ -z {PROJECT_NAME} ]]; then
  echo "no token given as second argument"
  exit -1
fi

REF_BRANCH='devel'

echo "create pipeline for ${PROJECT_NAME} on branch ${REF_BRANCH}"

RES=$(curl --request POST --header "PRIVATE-TOKEN: ${TOKEN}" "https://gitlab.com/api/v4/projects/srrg-software%2F${PROJECT_NAME}/pipeline?ref=${REF_BRANCH}")

echo $RES
