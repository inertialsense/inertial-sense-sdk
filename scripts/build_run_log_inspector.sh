#!/bin/bash

pushd "$(dirname "$(realpath $0)")" > /dev/null

source ./lib/echo_color.sh
source ./lib/results_build.sh
source ./lib/results_tests.sh

###############################################################################
#  Builds and Tests
###############################################################################

build_header "logInspector"
./build_log_inspector.sh
build_footer $?

pushd ../python/logInspector
python3 logInspectorInternal.py
popd

# echo "BUILD" $BUILD_EXIT_CODE
# echo "TESTS" $TESTS_EXIT_CODE
# echo "BOTH " $((BUILD_EXIT_CODE+TESTS_EXIT_CODE))

popd > /dev/null

# Return results: 0 = pass, 0 != fail
exit $((BUILD_EXIT_CODE+TESTS_EXIT_CODE))
