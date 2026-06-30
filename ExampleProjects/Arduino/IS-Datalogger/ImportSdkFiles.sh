#!/bin/bash

# This script copies needed SDK files into the Arduino project.

# rm -rf IS-SDK.zip ~/Arduino/libraries/src && zip -r IS-SDK.zip src -x "src/libusb*" -x "src/yaml-cpp*" -x "src/ISBoot*" -x "src/ISDFU*"

mkdir -p ~/Arduino/libraries/IS_SDK ~/Arduino/libraries/IS_SDK/core
cp ../../../src/{data_sets.h,data_sets.c,ISComm.h,ISComm.c,ISConstants.h,rtk_defines.h} ~/Arduino/libraries/IS_SDK
cp ../../../src/core/{base_port.c,base_port.h,msg_logger.h,types.h} ~/Arduino/libraries/IS_SDK/core
