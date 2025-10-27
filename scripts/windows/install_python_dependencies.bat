@echo off 

:: Set SDK_DIR as directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
call %SDK_DIR%\scripts\lib\activate_python_venv.bat

echo .
echo Installing Python dependencies C...
echo .

python -m pip install --upgrade pip
python -m pip install gitpython requests ruamel.yaml semver setuptools PyYAML
