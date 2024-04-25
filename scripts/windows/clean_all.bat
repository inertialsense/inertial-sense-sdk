@echo off 
for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi
setlocal
call :init_build_tools

REM MSBUILD_EXECUTABLE, MSBUILD_OPTIONS and NMAKE_EXECUTABLE are set in init_build_tools
set MSBUILD_OPTIONS=/maxcpucount:7 /p:MP=7 /t:Clean /p:Configuration=Release


::###############################################################################
::  Builds and Tests
::###############################################################################

@REM Remove build directory
rd /S /Q %~dp0\build 2> NUL

call :clean_header "IS_SDK_lib"
rd /S /Q %~dp0..\build 2> NUL
rd /S /Q %~dp0..\out 2> NUL
call :clean_footer

call :clean_header "cltool"
rd /S /Q %~dp0..\cltool\build 2> NUL
rd /S /Q %~dp0..\cltool\out 2> NUL
call :clean_footer

call :clean_header "IS_SDK_Examples"
rd /S /Q %~dp0..\ExampleProjects\build 2> NUL
rd /S /Q %~dp0..\ExampleProjects\out 2> NUL
call :clean_footer

call :clean_header "LogInspector"
call clean_python_cache.bat
call clean_log_inspector.bat
call :clean_footer

rem call :clean_header "Unit Tests Alt"
rem %MSBUILD_EXECUTABLE% "%SDK_DIR%\cpp\utilities\UnitTests\InertialSenseUnitTests.sln" %MSBUILD_OPTIONS%
rem call :clean_footer
rem call :tests_header "Unit Tests Alt"
rem "%SDK_DIR%\cpp\utilities\UnitTests\x64\Release\InertialSenseUnitTests_Release.exe"
rem call :tests_footer

@REM call :clean_directory "cltool" "%SDK_DIR%\SDK\cltool\VS_project\x64"


::###############################################################################
::  Results Summary
::###############################################################################
call :clean_results

:finish

:: Wait for any key press
rem pause
:: Pause for 3 seconds
ping localhost -n 3 >nul

:: Return results: 0 = pass, 1 = fail
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
