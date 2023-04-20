CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

project(ISLogReaderExample)

if (!WIN32)
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
endif()

if(BUILD_INERTIAL_SENSE_SDK)	# Build w/ SDK

add_executable(ISLogReaderExample ISLogReaderExample.cpp)
target_link_libraries(ISLogReaderExample InertialSense)

else()	# Build only this project

FILE(GLOB SOURCES_ALL
	ISLogReaderExample.cpp
	../../src/*.c
	../../src/*.cpp
	../../src/*.h
	../../hw-libs/bootloader/bootloaderShared.c
)

add_executable(ISLogReaderExample ${SOURCES_ALL})

find_library(LIBUSB_LIBRARY
	NAMES usb-1.0
	PATH_SUFFIXES "lib" "lib32" "lib64"
)

if(WIN32)
	add_definitions(-DYAML_CPP_STATIC_DEFINE)
	target_include_directories(ISLogReaderExample PUBLIC
		../../src/libusb/msvc
	)
else()
	target_include_directories(ISLogReaderExample PUBLIC
		../../src/libusb/linux
	)
endif()

target_include_directories(ISLogReaderExample PUBLIC
	../../src
	../../src/libusb/libusb
	../../src/libusb
)

if (WIN32)
	target_link_directories(ISLogReaderExample PRIVATE ../../src/libs)
	target_link_libraries(ISLogReaderExample)
else()
	target_link_libraries(ISLogReaderExample pthread ${LIBUSB_LIBRARY} m)
endif()

endif()
