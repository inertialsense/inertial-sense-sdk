#!/bin/bash

pushd "$(dirname "$(realpath $0)")" > /dev/null

source ./build_test_cmake.sh

###############################################################################
#  Builds and Tests
###############################################################################

build_cmake "cltool" ../cltool "$@"

./run_cltool.sh

# echo "BUILD" $BUILD_EXIT_CODE
# echo "TESTS" $TESTS_EXIT_CODE
# echo "BOTH " $((BUILD_EXIT_CODE+TESTS_EXIT_CODE))

popd > /dev/null

# Return results: 0 = pass, 0 != fail
exit $((BUILD_EXIT_CODE+TESTS_EXIT_CODE))
