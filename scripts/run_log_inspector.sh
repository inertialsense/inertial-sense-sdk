#!/usr/bin/env bash

source "$(dirname "$(realpath $0)")/lib/python_venv.sh" # Load python virtual enviroment must be ran before pushd
pushd "$(dirname "$(realpath $0)")" > /dev/null

python3 ../python/logInspector/logInspectorInternal.py

popd > /dev/null
