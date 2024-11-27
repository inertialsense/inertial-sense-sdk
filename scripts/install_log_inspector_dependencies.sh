#!/bin/bash
cd "$(dirname "$(realpath $0)")" > /dev/null
source lib/activate_python_venv.sh

sudo apt install -y python3 python3-pip
python3 -m pip install -U pip # update pip3 to latest version
python3 -m pip install setuptools wheel
python3 -m pip install pytest pybind11

./build_log_inspector.sh

source ~/.bashrc
