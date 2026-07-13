@echo off
setlocal

:: Resolve repo root (two levels up from scripts\windows\)
for %%i in ("%~dp0..\..") do set "SDK_DIR=%%~fi"

:: Activate the venv (no-op if already active)
call "%SDK_DIR%\scripts\lib\activate_python_venv.bat"

:: Delegate to the Python implementation
python "%SDK_DIR%\scripts\generate_doxygen.py" %*
set "ec=%ERRORLEVEL%"
endlocal & exit /b %ec%
