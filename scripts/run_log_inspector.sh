#!/usr/bin/env bash

pushd "$(dirname "$(realpath $0)")" > /dev/null

python3 ../python/logInspector/logInspectorInternal.py

popd > /dev/null
