@echo off

echo .
set "FORCE_DEPS=0"
for %%A in (%*) do (
	if /I "%%~A"=="--force" set "FORCE_DEPS=1"
	if /I "%%~A"=="--force-deps" set "FORCE_DEPS=1"
)

echo Installing python dependencies...
echo .

:: Set SDK_DIR as directory path
for %%i in (%~dp0..\..) do SET SDK_DIR="%%~fi"
call %SDK_DIR%\scripts\lib\activate_python_venv.bat

set "DEPS_MARKER=%venv_path%\deps_installed.txt"
if "%FORCE_DEPS%"=="0" if exist "%DEPS_MARKER%" (
	echo Dependencies already installed. Use --force-deps to reinstall.
	goto :eof
)

python -m pip install --upgrade pip
IF ERRORLEVEL 1 EXIT /B %ERRORLEVEL%
python -m pip install gitpython requests ruyaml semver setuptools pybind11 pygithub rich
IF ERRORLEVEL 1 EXIT /B %ERRORLEVEL%

echo gitpython requests ruyaml semver setuptools pybind11 pygithub rich> "%DEPS_MARKER%"
