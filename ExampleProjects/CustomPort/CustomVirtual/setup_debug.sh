#!/bin/bash

pushd "$(dirname "$(realpath $0)")" > /dev/null

mkdir -p build
rm -rf build/*
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j 7

popd > /dev/null
