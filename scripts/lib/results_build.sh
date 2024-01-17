#!/bin/bash

pushd "$(dirname "$(realpath $0)")" > /dev/null

LIB_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source ${LIB_DIR}/echo_color.sh

popd > /dev/null

BUILD_EXIT_CODE=0
LAST_BUILD_SUCCESS=0
BUILD_FAILURES="" 
BUILD_SUCCESS=""

function build_header() {
  build_name="$1"
  echo_build "=========================================="
  echo_build " BUILD:  ${build_name}"
  echo_build "=========================================="
}

function build_footer() {
  if [ $1 -eq 0 ]; then
    echo_green "[BUILD: ${build_name} - Passed]"
    BUILD_SUCCESS="${BUILD_SUCCESS}${build_name}, "
    LAST_BUILD_SUCCESS=1
    # if [ $2 -eq 0 ]; then
    # fi
  else
    echo_red "[***** BUILD: ${build_name} - FAILED *****]"
    BUILD_EXIT_CODE=1
    BUILD_FAILURES="${BUILD_FAILURES}${build_name}, "
    LAST_BUILD_SUCCESS=0
  fi
  echo ""
}

# This function removes the child directory and fails if parent direct does not exist. 
function clean_directory() {
  build_name=$1
  directory=$2
  root_path=$(dirname $2)
  echo_blue "=========================================="
  echo_blue " CLEAN ${build_name}"
  echo_blue "==========================================" 
  if [ -d "$root_path" ]; then
    rm -rf ${directory}
    echo_green "[Passed]"
    BUILD_SUCCESS="${BUILD_SUCCESS}${build_name}, "
  else
    echo "Path not found: $root_path"
    echo_red "[Failed]"
    BUILD_EXIT_CODE=1
    BUILD_FAILURES="${BUILD_FAILURES}${build_name}, "
  fi
}

