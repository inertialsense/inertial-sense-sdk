#!/usr/bin/env bash
cd "$(dirname "$(realpath $0)")" > /dev/null
source lib/activate_python_venv.sh

python3 ../python/logInspector/logInspector.py
