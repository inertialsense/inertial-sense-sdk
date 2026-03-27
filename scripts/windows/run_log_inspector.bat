@echo off
setlocal

echo Build Log Inspector
echo.

:: Set SDK_DIR to the repo root (two levels up from this script)
for %%i in ("%~dp0..\..") do set SDK_DIR="%%~fi"

:: Activate the Python virtual environment
call %SDK_DIR%\scripts\lib\activate_python_venv.bat || exit /b

:: Build Log Inspector locally
python %SDK_DIR%\scripts\build_log_inspector.py --run --no-build %*

:: Propagate the exit code
set "ec=%ERRORLEVEL%"
endlocal & exit /b %ec%
