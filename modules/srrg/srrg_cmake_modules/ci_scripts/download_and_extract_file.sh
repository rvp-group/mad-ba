#!/bin/bash
#ds this script is intended to be sourced!

#ds check input parameters
if [ "$#" -ne 3 ]; then
  echo "ERROR: call as $0 URL TARGET_FOLDER FILENAME"
  exit -1
fi
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
echo -e "\e[1;96mbash version: ${BASH_VERSION}\e[0m"
URL="$1"
TARGET_FOLDER="$2"
FILENAME="$3"

#ds setup target folder
mkdir -p "$TARGET_FOLDER"
cd "$TARGET_FOLDER"

#ds download file
wget --no-verbose "$URL" --output-document "$FILENAME"

#ds check if extraction is required
if [[ $FILENAME == *".zip"* ]]; then
  #ds extract zip file
  echo -e "\e[1;96munzip ${FILENAME}\e[0m"
  unzip "$FILENAME"
  rm "$FILENAME"
elif [[ $FILENAME == *".tar.gz"* ]]; then
  #ds extract tarball
  echo -e "\e[1;96mtar xzf ${FILENAME}\e[0m"
  tar xzf "$FILENAME"
  rm "$FILENAME"
fi

ls -al
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
