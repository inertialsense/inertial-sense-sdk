@echo off

REM # This script copies needed SDK files into the Arduino project.

robocopy ..\..\..\src src\ISsdk data_sets.h ISComm.h ISComm.c ISConstants.h rtk_defines.h
