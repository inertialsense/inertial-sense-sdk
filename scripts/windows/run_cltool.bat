@echo off
::
:: Run cltool.exe with support for command line arguments
::

setlocal enabledelayedexpansion

:: Capture the script directory
for %%i in ("%~dp0\..\..\..\SDK\cltool") do set "CLTOOL_BASE_DIR=%%~fi"

set "cltool_dir=!CLTOOL_BASE_DIR!\build-release"

echo Attempting to run cltool from: !cltool_dir!

if not exist "!cltool_dir!" (
	echo Error: Directory not found: !cltool_dir!
	exit /b 1
)

cd /d "!cltool_dir!"

if not exist "cltool.exe" (
	echo Error: cltool.exe not found in !cltool_dir!
	exit /b 1
)

:: Run cltool with all provided arguments
cltool.exe %*

