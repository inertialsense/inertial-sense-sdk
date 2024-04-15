@echo off 
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
if "%~1" equ "" (
    @REM arg 1 was not passed
    setlocal
    call :init_build_tools
)

REM MSBUILD_EXECUTABLE, MSBUILD_OPTIONS and NMAKE_EXECUTABLE are set in init_build_tools

::###############################################################################
::  Builds and Tests
::###############################################################################

call :build_header "SDK Example Projects"
cd %~dp0..\ExampleProjects
cmake -S . -B ./build "-DCMAKE_BUILD_TYPE=Release" && cmake --build ./build --config Release -j 7
call :build_footer

@REM cd to imx/scripts/windows directory
cd %~dp0    

:finish

if "%~1" equ "" (
    @REM ###############################################################################
    @REM   Results Summary
    @REM ###############################################################################
    call :build_results

    :: Remove temporary file X 
    rem del /q X
    :: Wait for any key press
    pause
)

:: Return results: 0 = pass, 0 != fail
exit /b %EXIT_CODE%

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
