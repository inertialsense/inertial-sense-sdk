#!/bin/bash
cd "$(dirname "$(realpath $0)")" > /dev/null

# Return if non-zero error code
python3 build_log_inspector.py "$@" || exit $?
