#!/bin/bash

# This script copies needed SDK files into the Arduino project.

mkdir -p src/ISsdk
cp ../../../src/{data_sets.c,data_sets.h,ISComm.h,ISComm.c,ISConstants.h} src/ISsdk/.
