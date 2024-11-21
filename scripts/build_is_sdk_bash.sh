#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source ${SCRIPT_DIR}/lib/echo_color.sh
source ${SCRIPT_DIR}/lib/results_build.sh
source ${SCRIPT_DIR}/lib/results_tests.sh

pushd "$(dirname "$(realpath $0)")" > /dev/null

BUILD_TYPE=Release
CLEAN="false"

for arg in "$@"; do
    case $arg in
        -c|--clean)
            CLEAN="true"
            ;;
        -d|--debug)
            BUILD_TYPE=Debug
            ;;
    esac
done

pushd .. > /dev/null

if [ "${CLEAN}" == "true" ]; then
    echo -e "\n\n=== Running make clean... ==="
    make clean 2> /dev/null
    rm -rf build
    build_result=$?
else
    echo -e "\n\n=== Running make... (${BUILD_TYPE} ==="
    cmake -B build -S . -DCMAKE_BUILD_TYPE=${BUILD_TYPE} && cmake --build build --config ${BUILD_TYPE} -j `nproc`
    build_result=$?
fi

popd > /dev/null

# Return results: 0 = pass, 0 != fail
exit ${build_result}
