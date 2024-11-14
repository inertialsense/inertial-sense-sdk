@echo off 

echo Installing Log Inspector dependencies...
echo.

:: Set SDK_DIR as directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi

:: Install dependencies
pushd %SDK_DIR%\python\
pip3 install setuptools pybind11 wheel
popd 
if %errorlevel% neq 0 ( echo Error installing Log Inspector dependencies! & exit /b %errorlevel% )

pushd %SDK_DIR%\python\
pip3 install logInspector/
popd
if %errorlevel% neq 0 ( echo Error installing Log Inspector dependencies! & exit /b %errorlevel% )

pause
