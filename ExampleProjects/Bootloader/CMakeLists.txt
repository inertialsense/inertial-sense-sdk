CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

project(ISBootloaderExample)

if (!WIN32)
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
endif()

if(BUILD_INERTIAL_SENSE_SDK)	# Build w/ SDK

add_executable(ISBootloaderExample ISBootloaderExample.cpp)
target_link_libraries(ISBootloaderExample InertialSense)

else()	# Build only this project

FILE(GLOB SOURCES_ALL
	ISBootloaderExample.cpp
	../../src/*.c
	../../src/*.cpp
	../../src/*.h
	../../hw-libs/bootloader/bootloaderShared.c
)

add_executable(ISBootloaderExample ${SOURCES_ALL})

find_library(LIBUSB_LIBRARY
	NAMES usb-1.0
	PATH_SUFFIXES "lib" "lib32" "lib64"
)

if(WIN32)
	add_definitions(-DYAML_CPP_STATIC_DEFINE)
	target_include_directories(ISBootloaderExample PUBLIC
		../../src/libusb/msvc
	)
else()
	target_include_directories(ISBootloaderExample PUBLIC
		../../src/libusb/linux
	)
endif()

target_include_directories(ISBootloaderExample PUBLIC
	../../src
	../../src/libusb/libusb
	../../src/libusb
)

if (WIN32)
	target_link_directories(ISBootloaderExample PRIVATE ../../src/libs)
	target_link_libraries(ISBootloaderExample)
else()
	# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
	target_link_libraries(ISBootloaderExample pthread ${LIBUSB_LIBRARY} yaml-cpp m)
endif()

endif()


