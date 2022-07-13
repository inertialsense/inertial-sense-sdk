CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)

project(ISBootloaderExample)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")


if(BUILD_INERTIAL_SENSE_SDK)	# Build w/ SDK

add_executable(ISBootloaderExample ISBootloaderExample.c)
target_link_libraries(ISBootloaderExample InertialSense)

else()	# Build only this project

add_executable(ISBootloaderExample
	ISBootloaderExample.c
	../../src/data_sets.c
	../../src/ISBootloaderThread.cpp
	../../src/ISBootloaderDFU.c
	../../src/ISBootloaderSAMBA.c
	../../src/ISBootloaderISB.c
	../../src/ISBootloader.c
	../../hw-libs/bootloader/bootloaderShared.c
	../../src/ihex.c
	../../src/ISComm.c
	../../src/serialPort.c
	../../src/serialPortPlatform.c
)
target_link_libraries(ISBootloaderExample m)

find_library(LIBUSB_LIBRARY
	NAMES usb-1.0
	PATH_SUFFIXES "lib" "lib32" "lib64"
)

target_include_directories(ISBootloaderExample PUBLIC
	../../src
	../../src/libusb
)

target_link_libraries(ISBootloaderExample ${LIBUSB_LIBRARY})

endif()


