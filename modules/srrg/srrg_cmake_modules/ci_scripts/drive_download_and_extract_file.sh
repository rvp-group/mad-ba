#!/bin/bash
#ds this script is intended to be sourced!

#ds check input parameters
if [ "$#" -ne 2 ]; then
  echo "ERROR: call as $0 GOOGLE_DRIVE_ID TARGET_FOLDER"
  exit -1
fi
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
echo -e "\e[1;96mbash version: ${BASH_VERSION}\e[0m"
GOOGLE_DRIVE_ID="$1"
TARGET_FOLDER="$2"
GOOGLE_DRIVE_URL="https://drive.google.com/uc?export=download"

#ds download and extract test dataset into current folder
mkdir -p "$TARGET_FOLDER"
cd "$TARGET_FOLDER"

#ds SED magic copied from: https://stackoverflow.com/questions/25010369/wget-curl-large-file-from-google-drive
#ds download large file from google drive (with cookie buffering)
FILENAME="$(curl -sc /tmp/gcokie "${GOOGLE_DRIVE_URL}&id=${GOOGLE_DRIVE_ID}" | grep -o '="uc-name.*</span>' | sed 's/.*">//;s/<.a> .*//')"  
GET_CODE="$(awk '/_warning_/ {print $NF}' /tmp/gcokie)"  
curl -Lb "/tmp/gcokie" "${GOOGLE_DRIVE_URL}&confirm=${GET_CODE}&id=${GOOGLE_DRIVE_ID}" -o "${FILENAME}"

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
cd ".."
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
