#!/bin/bash

pushd "$(dirname "$(realpath $0)")" > /dev/null

source ./lib/echo_color.sh
source ./lib/results_build.sh

###############################################################################
#  Clean
###############################################################################

build_header "LogInspector"
./build_log_inspector.sh --clean
build_footer $?

clean_directory "cltool"                ../cltool/build
clean_directory "SDK_Examples"          ../ExampleProjects/build
clean_directory "SDK_Unit_Tests"        ../tests/build

clean_directory "libInertialSenseSDK"   ../CMakeFiles
pushd ..
rm -rf cmake-build-debug CMakeCache.txt cmake_install.cmake libInertialSenseSDK.a
popd

rm -rf build




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
