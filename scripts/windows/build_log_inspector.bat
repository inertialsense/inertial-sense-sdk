@echo off

echo Build Log Inspector
echo.

:: Set SDK_DIR as  directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
call %SDK_DIR%\scripts\lib\activate_python_venv.bat

:: Build SDK cpp needed by LogInspector
call %SDK_DIR%\scripts\windows\build_is_sdk.bat

:: Build Log Inspector locally
pushd %SDK_DIR%\python\logInspector
python setup.py build_ext --inplace
popd
if %errorlevel% neq 0 ( echo Error building Log Inspector locally! & exit /b %errorlevel% )

