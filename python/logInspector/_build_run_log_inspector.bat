@echo off

echo Building and Run Log Inspector
echo.

cd ../

python -m pip install logInspector/

cd logInspector

python setup.py build_ext --inplace

python.exe logInspectorInternal.py

