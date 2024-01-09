#!/bin/bash

pushd "$(dirname "$(realpath $0)")" > /dev/null

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source ${SCRIPT_DIR}/lib/echo_color.sh
source ${SCRIPT_DIR}/lib/results_build.sh
source ${SCRIPT_DIR}/lib/results_tests.sh

BUILD='false'
TEST='false'

for i in "$@"; do
  case "$1" in
    -b|--build)
      BUILD='true'
      shift
      ;;
    -t|--test)
      TEST='true'
      shift
      ;;
  esac
done

function build_cmake() {
  testname="$1"
  cmakelists_dir="$2"

  pushd ${cmakelists_dir} > /dev/null
  build_header "${testname}"
  mkdir -p build
  pushd build > /dev/null
  cmake .. -DCMAKE_BUILD_TYPE=Release && make -j`nproc` -l`nproc`
  build_result=$?
  build_footer $build_result
  popd > /dev/null
  popd > /dev/null
  return $build_result
}

function test_cmake() {
  testname="$1"
  cmakelists_dir="$2"
  execname="run_tests"
  if ! [ -z "$3" ]
  then
      execname="$3"
  fi
  arguments="${@:4}" # All remaining arguments

  pushd ${cmakelists_dir} > /dev/null
  pushd build > /dev/null
  tests_header "${testname}"
  ./${execname} ${arguments}
  test_result=$?
  tests_footer $test_result
  popd > /dev/null
  popd > /dev/null
  return $test_result
}

function build_test_cmake() {
  build_cmake "$1" "$2" && test_cmake "$1" "$2"
}

# Options were shifted out earlier, so use $1 and $2
if [ ${BUILD} == 'true' ]; then
  if build_cmake "$1" "$2"; then
    : # success
  else
    popd > /dev/null
    exit 1  # error
  fi
fi
if [ ${TEST} == 'true' ]; then
  if test_cmake "$1" "$2"; then
    : # success
  else
    popd > /dev/null
    exit 1  # error
  fi
fi

popd > /dev/null
