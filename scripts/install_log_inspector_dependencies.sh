#!/bin/bash
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }

source "$(dirname "$(realpath $0)")/lib/python_venv.sh" # Load python virtual enviroment must be ran before pushd
pushd "$(dirname "$(realpath $0)")" > /dev/null

echo_blue "==============================================="
echo_blue " Install Dependencies: Log Inspector           "
echo_blue "==============================================="

sudo apt install -y python3 python3-pip python3-setuptools
python3 -m pip install -U pip # update pip3 to latest version

./install_pybind.sh

./build_log_inspector.sh

source ~/.bashrc

popd > /dev/null
