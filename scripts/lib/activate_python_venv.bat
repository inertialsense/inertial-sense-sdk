@echo off
REM This file must be run using the "call" command to set up the Python virtual environment

REM Get the directory of this script
for %%I in ("%~dp0") do set "LIB_DIR=%%~fI"

REM Call the Python script to get the virtual environment path
for /f "delims=" %%A in ('python "%LIB_DIR%python_venv.py"') do set "venv_path=%%A"

REM Activate the virtual environment
if not exist "%venv_path%\Scripts\activate.bat" (
    echo Error: failed to set up Python virtual environment 1>&2
    exit /b 1
)
call "%venv_path%\Scripts\activate.bat"

echo Activated virtual environment: %venv_path%
