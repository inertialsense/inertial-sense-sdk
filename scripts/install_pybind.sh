#!/bin/bash
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }

pushd "$(dirname "$(realpath $0)")" > /dev/null
source build_test_cmake.sh

python3 -m pip install pytest pybind11

source ~/.bashrc
popd > /dev/null
