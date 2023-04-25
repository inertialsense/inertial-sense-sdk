#!/bin/bash
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }

pushd "$(dirname "$(realpath $0)")" > /dev/null

echo_blue "==============================================="
echo_blue " Install Dependencies: Log Inspector           "
echo_blue "==============================================="

sudo apt install -y python3 python3-pip python3-setuptools
/usr/bin/python3 -m pip install -U pip # update pip3 to latest version

pushd ../python > /dev/null
pip3 install logInspector/
pushd logInspector > /dev/null
/usr/bin/python3 setup.py build_ext --inplace
popd > /dev/null
popd > /dev/null

source ~/.bashrc

popd > /dev/null
