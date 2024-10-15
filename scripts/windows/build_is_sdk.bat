@echo off 
setlocal

for %%i in (%~dp0..\..) do SET SDK_DIR=%%~fi

set CLEAN=0
set BUILD_TYPE=Release
set CMAKE_CXX_FLAGS=
set EXIT_CODE=0

:parse_args
FOR %%a IN (%*) DO (
    IF "%%a"=="-c" (
        set CLEAN=1
    ) ELSE IF "%%a"=="--clean" (
        set CLEAN=1
    ) ELSE IF "%%a"=="-d" (
        SET BUILD_TYPE=Debug
    ) ELSE IF "%%a"=="--debug" (
        SET BUILD_TYPE=Debug
    ) ELSE IF "%%a"=="-i" (
        SET CMAKE_CXX_FLAGS=%CMAKE_CXX_FLAGS%"-DUSE_IS_INTERNAL=1"
    ) ELSE IF "%%a"=="--internal" (
        SET CMAKE_CXX_FLAGS=%CMAKE_CXX_FLAGS%"-DUSE_IS_INTERNAL=1"
    )
)

:build_SDK
cd %~dp0..\..
IF %CLEAN%==1 (
    echo.
    echo === Cleaning SDK ===
    rd /S /Q build 2> NUL
    rd /S /Q out 2> NUL
) ELSE (
    echo.
    echo === Building SDK %BUILD_TYPE% %CMAKE_CXX_FLAGS% ===
    @REM cmake -S . "-DCMAKE_BUILD_TYPE=Release" && cmake --build . --config Release -j 7
    cmake -S . -B ./build "-DCMAKE_BUILD_TYPE=%BUILD_TYPE%" "-DCMAKE_CXX_FLAGS=%CMAKE_CXX_FLAGS%" && cmake --build ./build --config %BUILD_TYPE% -j 7
    SET "EXIT_CODE=%ERRORLEVEL%"
)

@REM cd to imx/scripts/windows directory
cd %~dp0    

:finish

:: Wait for any key press
@REM pause

:: Return results: 0 = pass, 0 != fail
exit /b %EXIT_CODE%
