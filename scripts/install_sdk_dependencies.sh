#!/bin/bash
cd "$(dirname "$(realpath $0)")" > /dev/null
source lib/activate_python_venv.sh

sudo apt install -y cmake unzip zip libyaml-cpp-dev libudev-dev
./install_python_dependencies.sh

if [[ -e ~/.bashrc ]]; then source ~/.bashrc; fi
