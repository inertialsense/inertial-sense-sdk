@echo off

echo Building and Run Log Inspector
echo.

:: Set SDK_DIR as  directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
cd %SDK_DIR%\python\

@REM python -m pip install logInspector/

pip3 install setuptools pybind11 wheel
python setup.py bdist_wheel sdist build_ext --inplace

:: Run Log Inspector

cd inertialsense/logInspector
python.exe logInspectorInternal.py

