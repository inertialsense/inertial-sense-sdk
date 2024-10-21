@echo off

echo Building Log Inspector
echo.

:: Set SDK_DIR as  directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
cd %SDK_DIR%\python\

@REM python -m pip install logInspector/

pip3 install setuptools pybind11 wheel
::pip3 install logInspector/
python setup.py bdist_wheel sdist build_ext --inplace

set ERROR_LVL=%errorlevel%

@REM timeout 10 /nobreak

@REM Set ERRORLEVEL now
@REM cmd /c exit %ERROR_LVL%

@REM Set ERRORLEVEL on exit
exit /b %ERROR_LVL%
