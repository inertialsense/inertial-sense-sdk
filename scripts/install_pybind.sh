#!/bin/bash
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }

pushd "$(dirname "$(realpath $0)")" > /dev/null
source ../SDK/scripts/build_test_cmake.sh

if [ ! -d ../../pybind11/build/ ] ; then
    pushd ../.. > /dev/null
    git clone https://github.com/pybind/pybind11.git
    build_cmake "pybind" ./pybind11
    popd > /dev/null
else
    echo_blue "==============================================="
    echo_blue " Using pre-built pybind11                      " 
    echo_blue "==============================================="
fi

/usr/bin/python3 -m pip install pytest

pushd ../../pybind11/build/ > /dev/null
sudo make install
popd > /dev/null

source ~/.bashrc

popd > /dev/null
