CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)

project(ISBootloaderCommonExample)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

if(BUILD_INERTIAL_SENSE_SDK)	# Build w/ SDK

	add_executable(ISBootloaderCommonExample ISBootloaderCommonExample.c)
	target_link_libraries(ISBootloaderCommonExample InertialSense)

else()	# Build only this project

	# set(SYSTEM_INCLUDE_DIRS "/usr/include/" "/usr/include/x86_64-linux-gnu")
	
	add_definitions(-DHAVE_UNISTD_H -DHAVE_NANOSLEEP)

	# Need to apt install libusb-dev
	find_path(LIBUSB_INCLUDE_DIR
		NAMES libusb.h
		PATH_SUFFIXES "include" "libusb" "libusb-1.0"
	)
	find_library(LIBUSB_LIBRARY
		NAMES usb-1.0
		PATH_SUFFIXES "lib" "lib32" "lib64"
	)

	add_executable(ISBootloaderCommonExample
		ISBootloaderCommonExample.c
		
		../../src/ISBootloaderCommon.c
		../../src/ISBootloaderDfu.c
		../../src/ISBootloaderLog.c
		
		../../src/ihex.c

		# ../../src/libusb/core.c
		# ../../src/libusb/descriptor.c
		# ../../src/libusb/hotplug.c
		# ../../src/libusb/io.c
		# ../../src/libusb/strerror.c
		# ../../src/libusb/sync.c
		# ../../src/libusb/os/events_posix.c
		# ../../src/libusb/os/events_posix.c
		# ../../src/libusb/os/linux_netlink.c
		# ../../src/libusb/os/linux_udev.c
		# ../../src/libusb/os/linux_usbfs.c
		# ../../src/libusb/os/threads_posix.c
	)

	target_include_directories(ISBootloaderCommonExample PUBLIC
		# ${SYSTEM_INCLUDE_DIRS}
		${LIBUSB_INCLUDE_DIR}
		../../src
		../../src/libusb
		../../src/libusb/os
	)

	target_link_libraries(ISBootloaderCommonExample
		${LIBUSB_LIBRARY}
	)

endif()


