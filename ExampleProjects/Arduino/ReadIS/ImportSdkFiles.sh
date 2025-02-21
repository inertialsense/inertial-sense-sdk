#!/bin/bash

# This script copies needed SDK files into the Arduino project.

set -x  # enable debugging

mkdir -p src/ISsdk
mkdir -p src/hw-libs/printf
mkdir -p src/ISsdk/core

cp ../../../src/{data_sets.c,data_sets.h,ISComm.h,ISComm.c,ISConstants.h,rtk_defines.h} src/ISsdk/.
cp ../../../hw-libs/printf/{printf.c,printf.h} src/hw-libs/printf/.
cp ../../../src/core/types.h src/ISsdk/core/.