CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

project(ISBootloaderExample)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")


if(BUILD_INERTIAL_SENSE_SDK)	# Build w/ SDK

add_executable(ISBootloaderExample ISBootloaderExample.cpp)
target_link_libraries(ISBootloaderExample InertialSense)

else()	# Build only this project

add_executable(ISBootloaderExample
	ISBootloaderExample.cpp
	../../src/data_sets.c
	
	../../hw-libs/firmwareSignatures.c
	../../src/ihex.c
	../../src/ISComm.c
	../../src/serialPort.c
	../../src/serialPortPlatform.c
	../../src/ISSerialPort.cpp
	# ../../src/ISBootloaderBase.cpp
	# ../../src/ISBootloaderThread.cpp
	# ../../src/ISBootloaderAPP.cpp
	# ../../src/ISBootloaderDFU.cpp
	../../src/ISUtilities.cpp
	../../src/ISStream.cpp
	../../src/ISFileManager.cpp
)
target_link_libraries(ISBootloaderExample m)

find_library(LIBUSB_LIBRARY
	NAMES usb-1.0
	PATH_SUFFIXES "lib" "lib32" "lib64"
)

target_include_directories(ISBootloaderExample PUBLIC
	../../src
	../../src/libusb/libusb
	../../src/libusb/msvc
)

target_link_libraries(ISBootloaderExample pthread ${LIBUSB_LIBRARY})

endif()


