@echo off 
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi

:: Return if non-zero error code
python ..\build_test_manager.py cltool ..\cltool %*

:: Check the exit code and exit with the same code if an error occurred
IF ERRORLEVEL 1 EXIT /B %ERRORLEVEL%