# instructions:
# install cmake, put it in your path for ease of use
# Linux: sudo apt install build-essential libudev-dev cmake
# Windows: Download and install msi from https://cmake.org/download/
# cd into cltool dir
# mkdir ./build
# create build files (on Windows this makes Visual Studio projects)
	# cmake -S . -B ./build
# build from command line
	# Windows: cmake --build ./build --target ALL_BUILD --config Release
	# Linux: cmake --build ./build

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)

project(cltool)

if (WIN32)
	add_definitions(-DNEED_PTHREAD_WINDOWS)
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
else()
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers")
endif()

if(INCLUDE_LUNA_DATA_SETS)	# run "cmake -DINCLUDE_LUNA_DATA_SETS=1 .."
    add_definitions(-DINCLUDE_LUNA_DATA_SETS)
endif()

if(BUILD_INERTIAL_SENSE_SDK) # Build w/ SDK

add_executable(cltool main.cpp)
target_link_libraries(cltool InertialSense)

else()	# Build only this project

SET(SOURCES_COMMON
main.cpp
	../src/cltool.cpp
	../src/cltool_main.cpp
	../src/com_manager.c
	../src/convert_ins.cpp
	../src/data_sets.c
	../src/DataChunk.cpp
	../src/DataChunkSorted.cpp
	../src/DataCSV.cpp
	../src/DataJSON.cpp
	../src/DataKML.cpp
	../src/DeviceLog.cpp
	../src/DeviceLogCSV.cpp
	../src/DeviceLogJSON.cpp
	../src/DeviceLogKML.cpp
	../src/DeviceLogSerial.cpp
	../src/DeviceLogSorted.cpp
	../src/ihex.c
	../src/InertialSense.cpp
	../src/ISBootloaderThread.cpp
	../src/ISBootloaderBase.cpp
    ../src/ISBootloaderISB.cpp
	../src/ISBootloaderDFU.cpp
	../src/ISBootloaderSAMBA.cpp
	../src/ISBootloaderAPP.cpp
	../hw-libs/bootloader/bootloaderShared.c
	../src/ISClient.cpp
	../src/ISComm.c
	../src/ISDataMappings.cpp
	../src/ISDisplay.cpp
	../src/ISEarth.c
	../src/ISFileManager.cpp
	../src/ISLogFile.cpp
	../src/ISLogger.cpp
	../src/ISLogStats.cpp
	../src/ISMatrix.c
	../src/ISPose.c
	../src/ISSerialPort.cpp
	../src/ISStream.cpp
	../src/ISTcpClient.cpp
	../src/ISTcpServer.cpp
	../src/ISUtilities.cpp
	../src/linked_list.c
	../src/message_stats.cpp
	../src/protocol_nmea.cpp
	../src/serialPort.c
	../src/serialPortPlatform.c
	../src/time_conversion.c
	../src/tinystr.cpp
	../src/tinyxml.cpp
	../src/tinyxmlerror.cpp
	../src/tinyxmlparser.cpp
)

SET(SOURCES_UNIX
	../src/libusb/libusb/os/events_posix.c
	../src/libusb/libusb/os/events_posix.h
	../src/libusb/libusb/os/linux_netlink.c
	../src/libusb/libusb/os/linux_udev.c
	../src/libusb/libusb/os/linux_usbfs.c
	../src/libusb/libusb/os/threads_posix.c
	../src/libusb/libusb/os/threads_posix.h
)
	
SET(SOURCES_WINDOWS
	../src/libusb/msvc/config.h
	../src/libusb/libusb/os/events_windows.c
	../src/libusb/libusb/os/events_windows.h
	../src/libusb/libusb/os/windows_common.c
	../src/libusb/libusb/os/windows_common.h
	../src/libusb/libusb/os/windows_usbdk.c
	../src/libusb/libusb/os/windows_usbdk.h
	../src/libusb/libusb/os/windows_winusb.c
	../src/libusb/libusb/os/windows_winusb.h
	../src/libusb/libusb/os/threads_windows.c
	../src/libusb/libusb/os/threads_windows.h
)

SET(SOURCES_OTHER
	../src/libusb/libusb/os/darwin_usb.c
	../src/libusb/libusb/os/darwin_usb.h
	../src/libusb/libusb/os/haiku_pollfs.cpp
	../src/libusb/libusb/os/haiku_usb.h
	../src/libusb/libusb/os/haiku_usb_backend.cpp
	../src/libusb/libusb/os/haiku_usb_raw.cpp
	../src/libusb/libusb/os/haiku_usb_raw.h
	../src/libusb/libusb/os/netbsd_usb.c
	../src/libusb/libusb/os/null_usb.c
	../src/libusb/libusb/os/openbsd_usb.c
	../src/libusb/libusb/os/sunos_usb.c
)

if(WIN32)
	add_executable(cltool ${SOURCES_COMMON} ${SOURCES_WIN32})
	target_include_directories(cltool PUBLIC
		../src/libusb/msvc
	)
else()
	add_executable(cltool ${SOURCES_COMMON} ${SOURCES_UNIX})
	target_include_directories(cltool PUBLIC
		../src/libusb/linux
	)
endif()

target_include_directories(cltool PUBLIC
	../src
	../src/libusb/libusb
	../src/libusb
)

target_link_directories(cltool PRIVATE ../src/libs)

if (WIN32)
target_link_libraries(cltool)
else()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
target_link_libraries(cltool pthread udev m)
endif()

endif()


