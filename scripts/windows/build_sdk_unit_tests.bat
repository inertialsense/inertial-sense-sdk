@echo off

echo Building SDK Unit Tests
echo.

:: Set SDK_DIR as  directory path
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
setlocal
call :init_build_tools

pushd "%SDK_DIR%\tests\VS_project\"
%MSBUILD_EXECUTABLE% "%SDK_DIR%\tests\VS_project\sdk_unit_tests.sln" %MSBUILD_OPTIONS%
set ERROR_LVL=%errorlevel%
@REM pushd %~dp0..\..\cpp\testing\unit_tests
@REM cmake -S . -B ./build "-DCMAKE_BUILD_TYPE=Release" && cmake --build ./build --config Release -j 7
popd

@REM Set ERRORLEVEL on exit
exit /b %ERROR_LVL%



::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:: Build Tools
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:init_build_tools
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:echo_blue
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:echo_green
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:echo_red
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:echo_cyan
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:echo_purple
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:echo_yellow
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:echo_white
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:echo_build
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:echo_tests  
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:build_header
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:build_footer
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:clean_header
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:clean_footer
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:tests_header
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:tests_footer
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:build_results
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:clean_results
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
:tests_results
    %SDK_DIR%\scripts\windows\build_tools.cmd %*
    goto :eof
