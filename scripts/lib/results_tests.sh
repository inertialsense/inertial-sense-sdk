#!/bin/bash

pushd "$(dirname "$(realpath $0)")" > /dev/null

LIB_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source ${LIB_DIR}/echo_color.sh

popd > /dev/null

TESTS_EXIT_CODE=0
LAST_TESTS_SUCCESS=0
TESTS_FAILURES=""
TESTS_SUCCESS=""

function tests_header() {
  test_name="$1"
  echo_tests "=========================================="
  echo_tests " TEST:  ${test_name}"
  echo_tests "=========================================="
}

function tests_footer() {
  if [ $? -eq 0 ] && [ $1 -eq 0 ]; then
    echo_green "[TEST: ${test_name} - Passed]"
    TESTS_SUCCESS="${TESTS_SUCCESS}${test_name}, "
    LAST_TESTS_SUCCESS=1
  else
    echo_red "[***** TEST: ${test_name} - FAILED *****]"
    TESTS_EXIT_CODE=1
    TESTS_FAILURES="${TESTS_FAILURES}${test_name}, "
    LAST_TESTS_SUCCESS=0
  fi
  echo ""
}
