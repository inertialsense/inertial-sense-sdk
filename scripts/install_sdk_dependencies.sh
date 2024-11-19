#!/bin/bash
cd "$(dirname "$(realpath $0)")" > /dev/null
source lib/activate_python_venv.sh

sudo apt install -y cmake unzip zip libyaml-cpp-dev libudev-dev
python3 -m pip install colorama

if [[ -e ~/.bashrc ]]; then source ~/.bashrc; fi
