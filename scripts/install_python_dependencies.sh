#!/bin/bash
cd "$(dirname "$(realpath $0)")" > /dev/null
source lib/activate_python_venv.sh

python3 -m pip install --upgrade pip
# Core build/test dependencies plus logInspector runtime deps
python3 -m pip install gitpython requests ruyaml semver setuptools pybind11 pygithub rich \
    pyyaml pyqt5 numpy matplotlib pandas scipy pyserial simplekml tqdm allantools
