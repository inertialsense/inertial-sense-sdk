CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

project(ISNtripRoverExample)

if (!WIN32)
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
endif()

if(BUILD_INERTIAL_SENSE_SDK)	# Build w/ SDK

add_executable(ISNtripRoverExample ISNtripRoverExample.cpp)
target_link_libraries(ISNtripRoverExample InertialSense)

else()	# Build only this project

FILE(GLOB SOURCES_ALL
	ISNtripRoverExample.cpp
	../../src/*.c
	../../src/*.cpp
	../../src/*.h
	../../hw-libs/bootloader/bootloaderShared.c
)

add_executable(ISNtripRoverExample ${SOURCES_ALL})

find_library(LIBUSB_LIBRARY
	NAMES usb-1.0
	PATH_SUFFIXES "lib" "lib32" "lib64"
)

if(WIN32)
	add_definitions(-DYAML_CPP_STATIC_DEFINE)
	target_include_directories(ISNtripRoverExample PUBLIC
		../../src/libusb/msvc
	)
else()
	target_include_directories(ISNtripRoverExample PUBLIC
		../../src/libusb/linux
	)
endif()

target_include_directories(ISNtripRoverExample PUBLIC
	../../src
	../../src/libusb/libusb
	../../src/libusb
)

if (WIN32)
	target_link_directories(ISNtripRoverExample PRIVATE ../../src/libs)
	target_link_libraries(ISNtripRoverExample)
else()
	target_link_libraries(ISNtripRoverExample pthread ${LIBUSB_LIBRARY} yaml-cpp)
endif()

endif()


