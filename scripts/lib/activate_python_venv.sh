#!/bin/bash

# This file must be sourced (not executed) to setup python virtual environment
LIB_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
venv_path=$(python3 ${LIB_DIR}/python_venv.py | tail -n 1)
source "${venv_path}/bin/activate"
echo "Activated virtual environment: ${venv_path}"

