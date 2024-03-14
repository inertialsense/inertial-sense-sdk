#!/bin/bash

pushd "$(dirname "$(realpath $0)")" > /dev/null

source ./build_test_cmake.sh

###############################################################################
#  Builds and Tests
###############################################################################

build_header "IS_SDK_lib"
./build_is_sdk.sh
build_footer $?

build_header "cltool"
./build_cltool.sh
build_footer $?

build_header "LogInspector"
./build_log_inspector.sh
build_footer $?

build_cmake "SDK_Unit_Tests"            ../tests IS-SDK_unit-tests
build_cmake "SDK_Examples"              ../ExampleProjects

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

if [ -n "$RELEASE_BUILD" ]; then 
    echo_yellow_title "Generated: RELEASE ${RELEASE_NAME}"
fi

popd > /dev/null


# echo "BUILD" $BUILD_EXIT_CODE
# echo "TESTS" $TESTS_EXIT_CODE
# echo "BOTH " $((BUILD_EXIT_CODE+TESTS_EXIT_CODE))

# Return results: 0 = pass, 0 != fail
# exit $((BUILD_EXIT_CODE+TESTS_EXIT_CODE))
