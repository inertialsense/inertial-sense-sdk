#!/bin/bash

args="${@:1}" # All arguments

pushd "$(dirname "$(realpath $0)")" > /dev/null

source ./lib/build_test_cmake.sh

build_cmake "SDK Unit Tests" ../tests ${args}

popd > /dev/null

# Return results: 0 = pass, 0 != fail
exit $((BUILD_EXIT_CODE+TESTS_EXIT_CODE))
