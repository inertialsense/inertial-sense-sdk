@echo off

echo Building and Run Log Inspector
echo.

:: Set SDK_DIR as  directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
cd %SDK_DIR%\python\

@REM python -m pip install logInspector/

cd logInspector
python setup.py build_ext --inplace

:: Run Log Inspector
python.exe logInspectorInternal.py

