#!/bin/bash
pushd "$(dirname "$(realpath $0)")" > /dev/null

../scripts/clean_all.sh
mkdir -p build
rm -rf build/*
cd build
cmake -DCMAKE_BUILD_TYPE=Debug .. \
    -DCMAKE_SYSROOT=/opt/intellian-ti-toolchain/armv7at2hf-neon-linux-gnueabi \
    -DCMAKE_FIND_ROOT_PATH=/opt/intellian-ti-toolchain/armv7at2hf-neon-linux-gnueabi \
    -DCMAKE_C_COMPILER=/opt/intellian-ti-toolchain/gcc-arm-9.2-2019.12-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-gcc \
    -DCMAKE_CXX_COMPILER=/opt/intellian-ti-toolchain/gcc-arm-9.2-2019.12-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-g++
make -j 7

popd > /dev/null
