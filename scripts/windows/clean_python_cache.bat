@echo off 

set "returndir=%cd%"
cd ../..

del /s /q *.pyc 2> NUL

cd %returndir%


