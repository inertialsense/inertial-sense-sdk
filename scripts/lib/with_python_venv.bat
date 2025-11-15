@echo off
REM Wrapper script that runs any command with Python virtual environment activated
REM Usage: call with_python_venv.bat <command> [args...]
REM Example: call with_python_venv.bat python build_all.py --clean

REM Get the directory of this script
for %%I in ("%~dp0") do set "LIB_DIR=%%~fI"

REM Activate the Python virtual environment  
call "%LIB_DIR%activate_python_venv.bat"

REM Run the provided command with all arguments
%*

REM Store the exit code before deactivating
SET COMMAND_EXIT_CODE=%ERRORLEVEL%

REM Deactivate the Python virtual environment
call deactivate 2>nul

REM Exit with the original command's exit code
EXIT /B %COMMAND_EXIT_CODE%