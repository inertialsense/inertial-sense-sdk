@echo off
setlocal

echo Build Log Inspector
echo.

:: Set SDK_DIR to the repo root (two levels up from this script)
for %%i in ("%~dp0..\..") do set "SDK_DIR=%%~fi"

:: Call with -r and forward all original args
call "%SDK_DIR%\scripts\windows\build_log_inspector.bat" -r %*

:: Propagate the exit code
set "ec=%ERRORLEVEL%"
endlocal & exit /b %ec%
