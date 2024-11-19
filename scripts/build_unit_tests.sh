#!/bin/bash
cd "$(dirname "$(realpath $0)")" > /dev/null

# Return if non-zero error code
python3 build_manager.py IS-SDK_unit-tests ../tests "$@" || exit $?