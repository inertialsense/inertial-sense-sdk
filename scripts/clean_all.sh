#!/bin/bash

pushd "$(dirname "$(realpath $0)")" > /dev/null

source ./lib/echo_color.sh
source ./lib/results_build.sh

###############################################################################
#  Clean
###############################################################################

build_header "IS_SDK_lib"
./build_is_sdk.sh --clean
build_footer $?

build_header "cltool"
./build_cltool.sh --clean
build_footer $?

build_header "LogInspector"
./build_log_inspector.sh --clean
build_footer $?

clean_directory "SDK_Examples"          ../ExampleProjects/build
clean_directory "SDK_Unit_Tests"        ../tests/build

build_header "EVB-2"
./build_firmware_evb2.sh --clean
build_footer $?


###############################################################################
#  Summary
###############################################################################

echo_build "=========================================="
echo_build " CLEAN RESULTS:"
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

popd > /dev/null
