CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)

project(ISBootloaderExample)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

if(BUILD_INERTIAL_SENSE_SDK)	# Build w/ SDK

	add_executable(ISBootloaderExample ISBootloaderExample.c)
	target_link_libraries(ISBootloaderExample InertialSense)

else()	# Build only this project

	# set(SYSTEM_INCLUDE_DIRS "/usr/include/" "/usr/include/x86_64-linux-gnu")
	set(LIBUSB_INCLUDE_DIR "/usr/include/libusb-1.0")
	set(DFU_UTIL_INCLUDE_DIR "../../hw-libs/dfu-util/src")
	set(LIBUSB_LIBRARY "/lib/x86_64-linux-gnu/libusb-1.0.so.0.1.0")
	
	add_definitions(-DHAVE_UNISTD_H -DHAVE_NANOSLEEP)

	add_executable(ISBootloaderExample
		ISBootloaderExample.c
		../../src/data_sets.c
		../../src/inertialSenseBootLoader_dfu.c
		../../src/ISComm.c
		../../src/serialPort_dfu.c
		../../src/serialPortPlatform_dfu.c
		../../hw-libs/dfu-util/src/dfu_util.c
		../../hw-libs/dfu-util/src/dfu_load.c
		../../hw-libs/dfu-util/src/dfu.c
		../../hw-libs/dfu-util/src/dfuse.c
		../../hw-libs/dfu-util/src/dfuse_mem.c
		../../hw-libs/dfu-util/src/dfu_file.c
		../../hw-libs/dfu-util/src/quirks.c
	)

	target_include_directories(ISBootloaderExample PUBLIC
		# ${SYSTEM_INCLUDE_DIRS}
		${LIBUSB_INCLUDE_DIR}
		${DFU_UTIL_INCLUDE_DIR})

	target_link_libraries(ISBootloaderExample m ${LIBUSB_LIBRARY})

endif()


