CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

project(ISCommunicationsExample)

if (!WIN32)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
endif()

if(BUILD_INERTIAL_SENSE_SDK)	# Build w/ SDK

add_executable(ISCommunicationsExample ISCommunicationsExample.c)
target_link_libraries(ISCommunicationsExample InertialSense)

else()	# Build only this project

FILE(GLOB SOURCES_ALL
    ISCommunicationsExample.c
    ../../src/data_sets.c
    ../../src/ISComm.c
    ../../src/ISMatrix.c
    ../../src/ISPose.c
    ../../src/serialPort.c
    ../../src/serialPortPlatform.c
)

add_executable(ISCommunicationsExample ${SOURCES_ALL})

if (WIN32)
    target_link_libraries(ISCommunicationsExample)
else()
    target_link_libraries(ISCommunicationsExample m)
endif()

endif()
