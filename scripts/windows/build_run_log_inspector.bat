@echo off
setlocal

echo Build Log Inspector
echo.

:: Set SDK_DIR to the repo root (two levels up from this script)
for %%i in ("%~dp0..\..") do set SDK_DIR="%%~fi"

:: Build Log Inspector locally
python %SDK_DIR%\scripts\build_log_inspector.py -r %*

:: Propagate the exit code
set "ec=%ERRORLEVEL%"
endlocal & exit /b %ec%
