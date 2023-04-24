# This example project does not use the CMakeCommon.txt file in the SDK/src folder
# If you don't want to follow that pattern, you can create your own cmake file
# The one define BUILD_INERTIAL_SENSE_SDK needs to be checked, and if on,
#  link the InertialSense library only

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.13)

project(ISCommunicationsExample)

if(NOT WIN32)
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
endif()

if(BUILD_INERTIAL_SENSE_SDK)	# Build w/ SDK

add_executable(ISCommunicationsExample ISCommunicationsExample.c)
target_link_libraries(ISCommunicationsExample InertialSense)

else()	# Build only this project

add_executable(ISCommunicationsExample
    ISCommunicationsExample.c
    ../../src/data_sets.c
    ../../src/ISComm.c
    ../../src/ISMatrix.c
    ../../src/ISPose.c
    ../../src/serialPort.c
    ../../src/serialPortPlatform.c
)

if(NOT WIN32)
	# Link in Linux specific packages
	target_link_libraries(ISCommunicationsExample udev m)
endif()

endif()
