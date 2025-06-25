@echo off 
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
call %SDK_DIR%\scripts\lib\activate_python_venv.bat

python %SDK_DIR%\scripts\build_all.py --clean %*

:: Check the exit code and exit with the same code if an error occurred
IF ERRORLEVEL 1 EXIT /B %ERRORLEVEL%
