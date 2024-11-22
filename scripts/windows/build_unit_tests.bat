@echo off 
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
call %SDK_DIR%\SDK\scripts\lib\activate_python_venv.bat

python %SDK_DIR%\scripts\build_manager.py IS-SDK_unit-tests %SDK_DIR%\tests %*

:: Check the exit code and exit with the same code if an error occurred
IF ERRORLEVEL 1 EXIT /B %ERRORLEVEL%
