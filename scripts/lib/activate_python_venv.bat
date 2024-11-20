@echo off
REM This file must be run using the "call" command to set up the Python virtual environment
setlocal

REM Get the directory of this script
for %%I in ("%~dp0") do set "LIB_DIR=%%~fI"

REM Call the Python script to get the virtual environment path
for /f "delims=" %%A in ('python "%LIB_DIR%python_venv.py"') do set "venv_path=%%A"

REM Activate the virtual environment
call "%venv_path%\Scripts\activate.bat"

echo Activated virtual environment: %venv_path%
endlocal
