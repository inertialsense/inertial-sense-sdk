#!/bin/bash

# **REMINDER**  If you modify this script, also make equivalent changes to update_repo_info.bat.

if [ $# -eq 0 ]
  then
    echo
    echo "ERROR: Supply path for repositoryInfo.h as argument!"
    echo
    exit
fi

# First argument is location of repositoryInfo.h
filepath="$1/repositoryInfo.h"

echo "Updating: ${filepath}"

# Get repo version
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source $SCRIPT_DIR/repo_version.sh

echo "#define REPO_DESCRIPTION \"${REPO_FULL_VERSION}\"">"${filepath}"
echo "#define REPO_HEAD_COUNT ${REPO_HEAD_COUNT}">>"${filepath}"
echo "#define REPO_VERSION_MAJOR ${REPO_VERSION_MAJOR}">>"${filepath}"
echo "#define REPO_VERSION_MINOR ${REPO_VERSION_MINOR}">>"${filepath}"
echo "#define REPO_VERSION_REVIS ${REPO_VERSION_REVIS}">>"${filepath}"
echo " ">>"${filepath}"

# Print file contents
cat "${filepath}"
