#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source ${SCRIPT_DIR}/lib/echo_color.sh
source ${SCRIPT_DIR}/lib/results_build.sh
source ${SCRIPT_DIR}/lib/results_tests.sh


function build_cmake() {

  local args=()
  local CLEAN='false'

  # Collect arguments that don't start with a dash in `args`
  while [[ $# -gt 0 ]]; do  
    case "$1" in
      -c|--clean)
        CLEAN='true'
        shift
        ;;
      -*)
        shift   # remove all arguments that start with a dash
        ;;
      *)
        args+=("$1")
        shift
        ;;
    esac
  done
  testname="${args[0]}"
  cmakelists_dir="${args[1]}"

  pushd ${cmakelists_dir} > /dev/null

  if [ "${CLEAN}" == 'true' ]; then
    echo -e "\n=== Running make clean... ==="
    rm -rf build
  else
    build_header "${testname}"
    mkdir -p build
    pushd build > /dev/null
    echo -e "\n\n=== Running make... (${BUILD_TYPE}) ==="
    pwd
    cmake .. -DCMAKE_BUILD_TYPE=${BUILD_TYPE} && make -j`nproc` -l`nproc`
    build_result=$?
    build_footer $build_result
    popd > /dev/null
  fi

  popd > /dev/null

  return $build_result
}


function test_cmake() {
  local args=()           # positional (non-dash) args: testname, cmakelists_dir, [execname]
  local gtest_opts=()     # dash-prefixed args to pass to gtest

  # Parse args
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --) # everything after -- goes to gtest
        shift
        while [[ $# -gt 0 ]]; do
          gtest_opts+=("$1")
          shift
        done
        ;;
      -*)
        gtest_opts+=("$1")
        shift
        ;;
      *)
        args+=("$1")
        shift
        ;;
    esac
  done

  local testname="${args[0]}"
  local cmakelists_dir="${args[1]}"
  local execname="${args[2]:-run_tests}"

  pushd "${cmakelists_dir}" >/dev/null
  pushd build >/dev/null

  tests_header "${testname}"
  "./${execname}" --gtest_color=yes "${gtest_opts[@]}"
  local test_result=$?
  tests_footer "$test_result"

  popd >/dev/null
  popd >/dev/null
  return "$test_result"
}


function build_test_cmake() {
  build_cmake "$@" && test_cmake "$@"
}


BUILD_TYPE=Release
BUILD='false'
CLEAN='false'
TEST='false'

for arg in "$@"; do
  case $arg in
    -b|--build)
      BUILD='true'
      ;;
    -c|--clean)
      CLEAN='true'
      ;;
    -d|--debug)
      BUILD_TYPE=Debug
      ;;
    -t|--test)
      TEST='true'
      ;;
  esac
done

if [ ${BUILD} == 'true' ]; then
  if build_cmake "$@"; then
    : # success
  else
    popd > /dev/null
    exit 1  # error
  fi
fi
if [ ${TEST} == 'true' ]; then
  if test_cmake "$@"; then
    : # success
  else
    popd > /dev/null
    exit 1  # error
  fi
fi

