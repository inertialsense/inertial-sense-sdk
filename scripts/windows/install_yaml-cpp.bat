@echo off

set YAMLCPP_VERSION=0.7.0
set YAMLCPP_FOLDER=yaml-cpp-%YAMLCPP_VERSION%
set YAMLCPP_ZIP=%YAMLCPP_FOLDER%.zip

set YAMLCPP_URL=https://github.com/jbeder/yaml-cpp/archive/refs/tags/%YAMLCPP_FOLDER%.zip

set YAMLCPP_BUILD_FOLDER=%YAMLCPP_FOLDER%\build
set YAMLCPP_LIB_FOLDER=%YAMLCPP_BUILD_FOLDER%\Release

echo Downloading yaml-cpp %YAMLCPP_VERSION%...
echo from %YAMLCPP_URL%
powershell -command "& { Invoke-WebRequest -Uri %YAMLCPP_URL% -OutFile %YAMLCPP_ZIP% }"

echo Extracting yaml-cpp %YAMLCPP_VERSION%...
powershell -command "& { Expand-Archive -Path %YAMLCPP_ZIP%}"

echo Building yaml-cpp %YAMLCPP_VERSION%...

echo cd %YAMLCPP_FOLDER%
cd %YAMLCPP_FOLDER%
echo cd yaml-cpp-%YAMLCPP_FOLDER%
cd yaml-cpp-%YAMLCPP_FOLDER%
mkdir build
cd build
cmake ..
cmake --build . --config Release

echo Installing yaml-cpp %YAMLCPP_VERSION%...
cd Release
echo %cd%

xcopy /y /q /i yaml-cpp.lib C:\Windows\System32
cd ../..
echo Current Directory: %cd%
xcopy "include" "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Tools\MSVC\14.29.30133\include" /E /H /C /I

echo Cleaning up...
cd ..\..
rmdir /s /q %YAMLCPP_FOLDER%
del /q %YAMLCPP_ZIP%

echo yaml-cpp %YAMLCPP_VERSION% has been installed successfully.
