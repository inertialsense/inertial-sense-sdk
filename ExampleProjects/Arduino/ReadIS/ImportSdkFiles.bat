@echo off

REM # This script copies needed SDK files into the Arduino project.

robocopy ..\..\..\src src\ISsdk data_sets.c data_sets.h ISComm.h ISComm.c ISConstants.h rtk_defines.h
robocopy ..\..\..\hw-libs\printf src\hw-libs\printf printf.h printf.c
robocopy ..\..\..\src\core src\ISsdk\core types.h
