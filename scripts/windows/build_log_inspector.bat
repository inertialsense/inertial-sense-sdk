@echo off

echo Build Log Inspector
echo.

:: Set SDK_DIR as directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi

:: Build SDK cpp needed by LogInspector
call %SDK_DIR%\scripts\windows\build_is_sdk.bat

:: Install dependencies
pushd %SDK_DIR%\python\
pip3 install setuptools pybind11 wheel
::pip3 install logInspector/
popd 
if %errorlevel% neq 0 ( echo Error installing Log Inspector dependencies! & exit /b %errorlevel% )

:: Build SDK python package
pushd %SDK_DIR%\python\
python setup.py bdist_wheel sdist build_ext --inplace
popd 
if %errorlevel% neq 0 ( echo Error building SDK python package! & exit /b %errorlevel% )

:: Build Log Inspector locally
pushd %SDK_DIR%\python
python setup.py build_ext --inplace
popd
if %errorlevel% neq 0 ( echo Error building Log Inspector locally! & exit /b %errorlevel% )

