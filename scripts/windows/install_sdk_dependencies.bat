@echo off

echo.
echo Installing SDK dependencies...
echo.

:: Set SDK_DIR as directory path
for %%i in ("%~dp0..\..") do SET SDK_DIR="%%~fi"
call %SDK_DIR%\scripts\lib\activate_python_venv.bat

vcpkg install gtest:x64-windows

install_python_dependencies.bat