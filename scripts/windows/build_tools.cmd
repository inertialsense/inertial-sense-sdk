@echo off
    setlocal enableextensions
    rem Not to be directly called
    exit /b 9009


::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:: Build Tools - Used to change 
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:init_build_tools
	set EXIT_CODE=0
	set LAST_BUILD_SUCCESS=0
	set LAST_TESTS_SUCCESS=0
	set BUILD_FAILURES=
	set BUILD_SUCCESS=
	set TESTS_FAILURES=
	set TESTS_SUCCESS=

	:: Locate MSBuild.exe
	set MSBUILD_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\Community\Msbuild\Current\Bin\MSBuild.exe"
    if exist %MSBUILD_EXECUTABLE% goto found_msbuild_executable
	set MSBUILD_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\Msbuild\Current\Bin\MSBuild.exe"
    if exist %MSBUILD_EXECUTABLE% goto found_msbuild_executable
    for /F "tokens* USEBACKQ" %%i in (MSBuild.exe) do ( set MSBUILD_EXECUTABLE=%%~$PATH:i )
    if exist %MSBUILD_EXECUTABLE% goto found_msbuild_executable
	echo Failed to locate MSBuild.exe!!!
	pause

:found_msbuild_executable
	:: Locate nmake.exe
	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.42.34433\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.41.34120\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.40.33807\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.39.33519\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.38.33130\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.36.32532\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.33.31629\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.32.31326\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\VC\Tools\MSVC\14.42.34433\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\VC\Tools\MSVC\14.41.34120\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\VC\Tools\MSVC\14.40.33807\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\VC\Tools\MSVC\14.39.33519\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\VC\Tools\MSVC\14.38.33130\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\VC\Tools\MSVC\14.36.32532\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\VC\Tools\MSVC\14.33.31629\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	set NMAKE_EXECUTABLE="C:\Program Files\Microsoft Visual Studio\2022\VC\Tools\MSVC\14.32.31326\bin\Hostx64\x64\nmake.exe"
    if exist %NMAKE_EXECUTABLE% goto found_nmake_executable

	echo Failed to locate nmake.exe!!!
	pause

:found_nmake_executable

    REM set MSBUILD_OPTIONS=/maxcpucount:7 /p:MP=7 /t:Build /p:Configuration=Release /p:Platform=x64
    set MSBUILD_OPTIONS=/maxcpucount:7 /p:MP=7 /t:Build /p:Configuration=Release /p:std=c++17

	:: Setup terminal for multi-colored text
	for /F "tokens=1,2 delims=#" %%a in ('"prompt #$H#$E# & echo on & for %%b in (1) do rem"') do (
	set ESC=%%b
	exit /B 0
	)
	exit /B 0

:: Functions to print in color
:echo_blue
	set _str=%*
	set _str=%_str:"=%
	echo %ESC%[94m%_str%%ESC%[0m
	exit /b

:echo_green
	set _str=%*
	set _str=%_str:"=%
	echo %ESC%[92m%_str%%ESC%[0m
	exit /b

:echo_red
	set _str=%*
	set _str=%_str:"=%
	echo %ESC%[91m%_str%%ESC%[0m
	exit /b

:echo_cyan
	set _str=%*
	set _str=%_str:"=%
	echo %ESC%[96m%_str%%ESC%[0m
	exit /b

:echo_purple
	set _str=%*
	set _str=%_str:"=%
	echo %ESC%[95m%_str%%ESC%[0m
	exit /b

:echo_yellow
	set _str=%*
	set _str=%_str:"=%
	echo %ESC%[93m%_str%%ESC%[0m
	exit /B

:echo_white
	set _str=%*
	set _str=%_str:"=%
	echo %ESC%[97m%_str%%ESC%[0m
	exit /B

:echo_build
	call :echo_blue %*
	exit /b

:echo_tests  
	call :echo_blue %*
	exit /b

:build_header
	set build_name=%*
	:: remove quotes
	set tmp_name=%build_name:"=%
	call :echo_build "=========================================="
	call :echo_build " BUILD:  %tmp_name%"
	call :echo_build "=========================================="
	exit /b

:build_footer
	@REM echo %ERRORLEVEL% "ERROR LEVEL"
	if %ERRORLEVEL% == 0 (
		call :echo_green "[BUILD: %tmp_name% - Passed]"
		set BUILD_SUCCESS=%BUILD_SUCCESS% %build_name:"=%,
		set LAST_BUILD_SUCCESS=1
	) else (
		call :echo_red "[***** BUILD: %tmp_name% - FAILED *****]"
		set EXIT_CODE=1
		set BUILD_FAILURES=%BUILD_FAILURES% %build_name:"=%,
		set LAST_BUILD_SUCCESS=0
	)
	echo.
	exit /b

:clean_header
	set build_name=%*
	:: remove quotes
	set name=%build_name:"=%
	call :echo_build "=========================================="
	call :echo_build " CLEAN:  %name%"
	call :echo_build "=========================================="
	exit /b

:clean_footer
	if %ERRORLEVEL% == 0 (
		call :echo_green "[Build Passed]"
		set BUILD_SUCCESS=%BUILD_SUCCESS% %build_name:"=%,
		set LAST_BUILD_SUCCESS=1
	) else (
		call :echo_red "[***** CLEAN FAILED *****]"
		set EXIT_CODE=1
		set BUILD_FAILURES=%BUILD_FAILURES% %build_name:"=%,
		set LAST_BUILD_SUCCESS=0
	)
	echo.
	exit /b

:tests_header
	set test_name=%*
	:: remove quotes
	set tmp_name=%test_name:"=%
	call :echo_tests "=========================================="
	call :echo_tests " TEST:  %tmp_name%"
	call :echo_tests "=========================================="
	exit /b

:tests_footer
	if %ERRORLEVEL% == 0 (
		call :echo_green "[TEST: %tmp_name% - Passed]"
		set TESTS_SUCCESS=%TESTS_SUCCESS% %test_name:"=%,
		set LAST_TESTS_SUCCESS=1
	) else (
		call :echo_red "[***** TEST: %tmp_name% - FAILED *****]"
		set EXIT_CODE=1
		set TESTS_FAILURES=%TESTS_FAILURES% %test_name:"=%,
		set LAST_TESTS_SUCCESS=0
	)
	echo.
	exit /b

:clean_directory
	set build_name=%~1
	set full_path=%~2
	set root_path=%~dp2
	:: remove quotes
	call :echo_blue "=========================================="
	call :echo_blue " CLEAN DIRECTORY:  %build_name%"
	call :echo_blue "=========================================="
	if exist %root_path% (
		if exist %full_path% (
			rmdir /s /q %full_path%
		)
		call :echo_green "[Passed]"
		set BUILD_SUCCESS=%BUILD_SUCCESS% %build_name:"=%,
		set LAST_BUILD_SUCCESS=1
		call :echo_green "%build_name%"
	) else (
		echo Path not found: %root_path%
		call :echo_red "[Failed]"
		set EXIT_CODE=1
		set BUILD_FAILURES=%BUILD_FAILURES% %build_name:"=%,
		set LAST_BUILD_SUCCESS=0
	)
	echo.
	exit /b

:build_results
	call :echo_build "=========================================="
	call :echo_build " BUILD RESULTS:"
	call :echo_build "=========================================="
	if NOT "[%BUILD_SUCCESS%]"=="[]" (
		call :echo_green "[PASSED]: %BUILD_SUCCESS:"=%"
	)
	if NOT "[%BUILD_FAILURES%]"=="[]" (
		call :echo_red "[FAILED]: %BUILD_FAILURES:"=%"
	)
	echo.
	exit /b

:clean_results
    call :echo_build "=========================================="
    call :echo_build " CLEAN RESULTS:"
    call :echo_build "=========================================="
    if NOT "[%BUILD_SUCCESS%]"=="[]" (
        call :echo_green "[PASSED]: %BUILD_SUCCESS:"=%"
    )
    if NOT "[%BUILD_FAILURES%]"=="[]" (
        call :echo_red "[FAILED]: %BUILD_FAILURES:"=%"
    )
    echo.  
	exit /b

:tests_results
	call :echo_tests "=========================================="
	call :echo_tests " TEST RESULTS:"
	call :echo_tests "=========================================="
	if NOT "[%TESTS_SUCCESS%]"=="[]" (
		call :echo_green "[PASSED]: %TESTS_SUCCESS:"=%"
	)
	if NOT "[%TESTS_FAILURES%]"=="[]" (
		call :echo_red "[FAILED]: %TESTS_FAILURES:"=%"
	)
	echo. 
	exit /b

