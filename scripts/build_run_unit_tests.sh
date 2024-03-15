#!/bin/bash

pushd "$(dirname "$(realpath $0)")" > /dev/null

./build_unit_tests.sh

../tests/build/IS-SDK_unit-tests 

popd > /dev/null

