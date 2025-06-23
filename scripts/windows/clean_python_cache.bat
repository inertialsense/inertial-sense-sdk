@echo off 

:: Set SDK_DIR as  directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi

set "BASE_DIR=%SDK_DIR%\python"

REM Recursively find and delete all __pycache__ directories
for /r "%BASE_DIR%" %%d in (.) do (
    if exist "%%d\__pycache__" (
        echo Deleting %%d\__pycache__
        rmdir /s /q "%%d\__pycache__"
    )
)

rd /S /Q %SDK_DIR%\python\build 2> NUL
