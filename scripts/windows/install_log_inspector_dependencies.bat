@echo off 

echo Installing Log Inspector dependencies...
echo.

:: Set SDK_DIR as directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
call %SDK_DIR%\scripts\lib\activate_python_venv.bat

:: Install dependencies
pip3 install setuptools pybind11 wheel
if %errorlevel% neq 0 ( echo Error installing Log Inspector dependencies! & exit /b %errorlevel% )

pip3 install %SDK_DIR%/python/
if %errorlevel% neq 0 ( echo Error installing Log Inspector dependencies! & exit /b %errorlevel% )

