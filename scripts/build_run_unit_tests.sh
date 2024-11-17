#!/bin/bash
pushd "$(dirname "$(realpath $0)")" > /dev/null

# Return if non-zero error code
python3 build_test_manager.py IS-SDK_unit-tests ../tests --test || exit $?

