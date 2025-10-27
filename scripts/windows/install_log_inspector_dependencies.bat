@echo off 

echo Installing Log Inspector dependencies...
echo.

:: Set SDK_DIR as directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
call %SDK_DIR%\scripts\lib\activate_python_venv.bat

:: Install dependencies
python.exe -m pip install setuptools pybind11 wheel  || (echo SDK python dependencies install failed & exit /b 1) 

python.exe -m pip install "%SDK_DIR%\python" || (echo SDK-python package install failed & exit /b 1)

