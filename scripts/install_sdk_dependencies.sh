#!/bin/bash
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }

pushd "$(dirname "$(realpath $0)")" > /dev/null

echo_blue "==============================================="
echo_blue " Install Dependencies: SDK                     "
echo_blue "==============================================="

sudo apt install -y cmake unzip zip libyaml-cpp-dev libudev-dev

source ~/.bashrc

popd > /dev/null
