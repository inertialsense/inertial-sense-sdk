#!/bin/bash
cd "$(dirname "$(realpath $0)")" > /dev/null
source lib/activate_python_venv.sh

python3 -m pip install setuptools pybind11 gitpython requests ruamel.yaml semver
