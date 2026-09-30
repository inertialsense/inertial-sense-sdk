#!/bin/bash
cd "$(dirname "$(realpath $0)")" > /dev/null
source lib/activate_python_venv.sh || exit 1

# Return if non-zero error code
python3 build_log_inspector.py "$@" || exit $?
