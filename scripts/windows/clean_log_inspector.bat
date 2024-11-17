@echo off

echo Clean Log Inspector
echo.

:: Set SDK_DIR as  directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi

rd /S /Q %SDK_DIR%\python\logInspector\build 2> NUL
rd /S /Q %SDK_DIR%\python\logInspector\__pycache__ 2> NUL
del /q %SDK_DIR%\python\logInspector\log_reader.*.pyd 2> NUL

REM timeout 5 /nobreak
