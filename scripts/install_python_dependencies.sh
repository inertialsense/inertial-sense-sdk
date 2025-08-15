#!/bin/bash
cd "$(dirname "$(realpath $0)")" > /dev/null
source lib/activate_python_venv.sh

python3 -m pip install --upgrade pip
python3 -m pip install gitpython pybind11 requests ruamel.yaml semver setuptools
