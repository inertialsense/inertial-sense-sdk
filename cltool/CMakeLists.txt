# instructions:
# install cmake, put it in your path for ease of use
# Linux: sudo apt install build-essential libudev-dev libusb-1.0-0-dev libyaml-cpp-dev cmake
# Windows: Download and install msi from https://cmake.org/download/
# cd into cltool dir
# mkdir ./build
# create build files (on Windows this makes Visual Studio projects)
	# cmake -S . -B ./build
# build from command line
	# Windows: cmake --build ./build --target ALL_BUILD --config Release
	# Linux: cmake --build ./build --config Release

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)

project(cltool)

if (WIN32)
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

FILE(GLOB SOURCES_ALL
	"main.cpp"
	"../src/*.c"
	"../src/*.cpp"
	"../src/*.h"
	"../hw-libs/bootloader/bootloaderShared.c"
)

add_executable(cltool ${SOURCES_ALL})

if(WIN32)
	add_definitions(-DYAML_CPP_STATIC_DEFINE)
	target_include_directories(cltool PUBLIC
		../src/libusb/msvc
	)
else()
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
target_link_libraries(cltool libusb-1.0.lib yaml-cpp.lib)
else()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
target_link_libraries(cltool libusb-1.0.a udev yaml-cpp m)
endif()

endif()
