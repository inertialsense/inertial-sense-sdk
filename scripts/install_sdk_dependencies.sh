#!/bin/bash
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }

source "$(dirname "$(realpath $0)")/lib/python_venv.sh" # Load python virtual enviroment must be ran before pushd
pushd "$(dirname "$(realpath $0)")" > /dev/null

echo_blue "==============================================="
echo_blue " Install Dependencies: SDK                     "
echo_blue "==============================================="

sudo apt install -y cmake unzip zip libyaml-cpp-dev libudev-dev
python3 -m pip install colorama

if [[ -e ~/.bashrc ]]; then source ~/.bashrc; fi

popd > /dev/null
