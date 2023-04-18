@echo off

set YAMLCPP_VERSION=0.7.0
set YAMLCPP_FOLDER=yaml-cpp-%YAMLCPP_VERSION%
set YAMLCPP_ZIP=%YAMLCPP_FOLDER%.zip

set YAMLCPP_URL=https://github.com/jbeder/yaml-cpp/archive/%YAMLCPP_VERSION%.zip

set YAMLCPP_BUILD_FOLDER=%YAMLCPP_FOLDER%\build
set YAMLCPP_LIB_FOLDER=%YAMLCPP_BUILD_FOLDER%\Release

echo Downloading yaml-cpp %YAMLCPP_VERSION%...
curl -L -o %YAMLCPP_ZIP% %YAMLCPP_URL%

echo Extracting yaml-cpp %YAMLCPP_VERSION%...
7z x %YAMLCPP_ZIP%

echo Building yaml-cpp %YAMLCPP_VERSION%...
mkdir %YAMLCPP_BUILD_FOLDER%
cd %YAMLCPP_BUILD_FOLDER%
cmake -G "Visual Studio 16 2019" -DCMAKE_BUILD_TYPE=Release .. > nul
cmake --build . --config Release > nul

echo Installing yaml-cpp %YAMLCPP_VERSION%...
xcopy /y /q /i %YAMLCPP_LIB_FOLDER%\libyaml-cpp.lib C:\Windows\System32
xcopy /y /q /i %YAMLCPP_FOLDER%\include C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Tools\MSVC\14.29.30133\include

echo Cleaning up...
cd ..\..
rmdir /s /q %YAMLCPP_FOLDER%
del /q %YAMLCPP_ZIP%

echo yaml-cpp %YAMLCPP_VERSION% has been installed successfully.
