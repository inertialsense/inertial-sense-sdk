@echo off
setlocal

echo Build Log Inspector
echo.

:: Set SDK_DIR to the repo root (two levels up from this script)
for %%i in ("%~dp0..\..") do set "SDK_DIR=%%~fi"

:: Activate the venv (ok if this is a no-op)
call "%SDK_DIR%\scripts\lib\activate_python_venv.bat"

:: Build SDK C++ (needed by LogInspector)
call "%SDK_DIR%\scripts\windows\build_is_sdk.bat" %*
if errorlevel 1 goto :fail

:: Build/run Log Inspector locally
python %SDK_DIR%\scripts\build_log_inspector.py %*
set "ec=%ERRORLEVEL%"
endlocal & exit /b %ec%

:fail
set "ec=%ERRORLEVEL%"
endlocal & exit /b %ec%
