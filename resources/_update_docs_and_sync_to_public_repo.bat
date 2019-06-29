@echo off

set SDKDIR=..\..\..\InertialSenseSDK
set COMDIR=..\..\..\inertialsense_serial_protocol

echo.
echo Prepare SDK for release.
::echo.

cd ..

echo.
echo Cleaning SDK
call :remove_sub_directories build
call :remove_sub_directories Debug
call :remove_sub_directories Release
call :remove_sub_directories .vs
call :remove_sub_directories .vs
call :remove_sub_directories IS_logs

cd resources

echo.
echo Update doxygen docs
call update_doxygen_docs.bat

cd ..

echo.
echo Sync SDK into %SDKDIR% repo
robocopy %SDKDIR%\.git %SDKDIR%tmp\.git /MIR /XA:H
robocopy . %SDKDIR% /MIR /XA:H
robocopy %SDKDIR%tmp\.git %SDKDIR%\.git /MIR /XA:H
::rmdir /S /Q %SDKDIR%tmp

cd src 

echo.
echo Sync SDK into %COMDIR% repo
robocopy . ..\%COMDIR% data_sets.c data_sets.h ISComm.c ISComm.h ISConstants.h

echo.
echo Sync inertial_sense_ros repo
cd ..\..\..\catkin_ws\src\inertial_sense_ros
update_and_sync_ros_node.bat

echo.
echo Done.
echo.

:: Sleep using ping for invalid ip and timeout
::ping 123.45.67.89 -n 1 -w 5000 > nul

:: Wait for user input
pause
:: Success
exit 0

:remove_sub_directories
echo - removing "%~1" directories
for /d /r "." %%a in (%~1\) do (
    if exist "%%a" rmdir /s /q "%%a"
)

