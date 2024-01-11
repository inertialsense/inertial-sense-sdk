#!/bin/bash

# **REMINDER**  If you modify this script, also make equivalent changes to update_build_info.bat.

if [ $# -eq 0 ]
  then
    echo
    echo "ERROR: Supply path for buildInfo.h as argument!"
    echo
    exit
fi

# First argument is location of buildInfo.h
filepath="$1/buildInfo.h"

echo "Updating: ${filepath}"

BUILD_NUMBER=0

if [ -f "${filepath}" ]; then
	BUILD_NUMBER="$(grep BUILD_NUMBER ${filepath} | cut -d" " -f3)"
fi

# Increment build number
BUILD_NUMBER=$(($BUILD_NUMBER + 1))

# Get date and time
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source $SCRIPT_DIR/build_date_time.sh

echo "#define BUILD_NUMBER ${BUILD_NUMBER}">${filepath}
echo "#define BUILD_DATE_YEAR ${BUILD_DATE_YEAR}">>${filepath}
echo "#define BUILD_DATE_MONTH ${BUILD_DATE_MONTH}">>${filepath}
echo "#define BUILD_DATE_DAY ${BUILD_DATE_DAY}">>${filepath}
echo "#define BUILD_TIME_HOUR ${BUILD_TIME_HOUR}">>${filepath}
echo "#define BUILD_TIME_MINUTE ${BUILD_TIME_MINUTE}">>${filepath}
echo "#define BUILD_TIME_SECOND ${BUILD_TIME_SECOND}">>${filepath}
echo "#define BUILD_TIME_MILLISECOND ${BUILD_TIME_MILLISECOND}">>${filepath}
echo " ">>${filepath}

# Print file contents
echo "$(cat ${filepath})"


