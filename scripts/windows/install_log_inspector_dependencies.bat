@echo off 

echo Installing Log Inspector dependencies...
echo.

:: Set SDK_DIR as  directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
cd %SDK_DIR%\python\

:: Install dependencies
pip install logInspector/
cd logInspector
python setup.py build_ext --inplace

pause
