#!/bin/bash
cd "$(dirname "$(realpath $0)")" > /dev/null
source lib/activate_python_venv.sh

# Return if non-zero error code
python3 build_all.py --clean "$@" || exit $?
