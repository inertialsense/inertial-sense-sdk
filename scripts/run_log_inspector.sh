#!/usr/bin/env bash
cd "$(dirname "$(realpath $0)")" > /dev/null
source lib/activate_python_venv.sh || exit 1

python3 ../python/inertialsense/logInspector/logInspector.py
