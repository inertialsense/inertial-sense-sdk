#!/bin/bash

pushd "$(dirname "$(realpath $0)")" > /dev/null

source ./build_test_cmake.sh

build_test_cmake "SDK Unit Tests" ../tests IS-SDK_unit-tests


###############################################################################
#  Summary
###############################################################################

echo_build "=========================================="
echo_build " BUILD RESULTS:"
echo_build "=========================================="
if [ -n "$BUILD_SUCCESS" ]
then
    echo_green "[PASSED]: $BUILD_SUCCESS"
fi
if [ -n "$BUILD_FAILURES" ]
then
    echo_red "[FAILED]: $BUILD_FAILURES"
fi
echo

echo_tests "=========================================="
echo_tests " TEST RESULTS:"
echo_tests "=========================================="
if [ -n "$TESTS_SUCCESS" ]
then
    echo_green "[PASSED]: $TESTS_SUCCESS"
fi
if [ -n "$TESTS_FAILURES" ]
then
    echo_red "[FAILED]: $TESTS_FAILURES"
fi
echo

if [ -n "$RELEASE_BUILD" ]; then
    echo_yellow_title "Generated: RELEASE ${RELEASE_NAME}"
fi

popd > /dev/null

exit $((BUILD_EXIT_CODE+TESTS_EXIT_CODE))
