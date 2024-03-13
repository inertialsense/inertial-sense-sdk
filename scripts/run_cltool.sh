#!/usr/bin/env bash

args="${@:1}" # All arguments

pushd "$(dirname "$(realpath $0)")" > /dev/null

../cltool/build/cltool ${args}

popd > /dev/null
