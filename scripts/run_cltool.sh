#!/usr/bin/env bash

pushd "$(dirname "$(realpath $0)")" > /dev/null

../cltool/build/cltool "$@"

popd > /dev/null
