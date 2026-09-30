#!/bin/bash

# This file must be sourced (not executed) to setup python virtual environment
LIB_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
venv_path=$(python3 ${LIB_DIR}/python_venv.py | tail -n 1)
if [ ! -f "${venv_path}/bin/activate" ]; then
    echo "Error: failed to set up Python virtual environment" >&2
    return 1
fi
source "${venv_path}/bin/activate"
echo "Activated virtual environment: ${venv_path}"

