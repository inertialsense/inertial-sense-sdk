@echo off

echo Build Log Inspector
echo.

:: Set SDK_DIR as  directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
call %SDK_DIR%\scripts\lib\activate_python_venv.bat

:: Build SDK cpp needed by LogInspector
call %SDK_DIR%\scripts\windows\build_is_sdk.bat %*

:: Build Log Inspector locally
python %SDK_DIR%\scripts\build_log_inspector.py %*

:: Check the exit code and exit with the same code if an error occurred
IF ERRORLEVEL 1 EXIT /B %ERRORLEVEL%
