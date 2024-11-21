#!/bin/bash
cd "$(dirname "$(realpath $0)")" > /dev/null
source lib/activate_python_venv.sh

python3 -m pip install pytest pybind11
source ~/.bashrc
