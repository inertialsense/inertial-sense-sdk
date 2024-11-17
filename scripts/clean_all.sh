#!/bin/bash
pushd "$(dirname "$(realpath $0)")" > /dev/null

# Return if non-zero error code
python3 build_all.py --clean || exit $?
