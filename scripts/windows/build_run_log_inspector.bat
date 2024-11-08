@echo off

echo Build and Run Log Inspector
echo.

:: Set SDK_DIR as  directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi

:: Build log Inspector
call %SDK_DIR%\scripts\windows\build_log_inspector.bat
if %errorlevel% neq 0 ( echo Error building Log Inspector! & exit /b %errorlevel% )

:: Run Log Inspector
python.exe %SDK_DIR%\python\inertialsense\logInspector\logInspectorInternal.py

