@echo off

echo Clean Log Inspector
echo.

:: Set SDK_DIR as  directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
cd %SDK_DIR%\python\logInspector\

rd /s /q build
del /q log_reader.*.pyd

REM timeout 5 /nobreak
