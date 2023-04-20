CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

project(ISAsciiExample)

if (!WIN32)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
endif()

if(BUILD_INERTIAL_SENSE_SDK)	# Build w/ SDK

add_executable(ISAsciiExample ISAsciiExample.c)
target_link_libraries(ISAsciiExample InertialSense)

else()	# Build only this project

add_executable(ISAsciiExample
    ISAsciiExample.c
    ../../src/data_sets.c
    ../../src/ISComm.c
    ../../src/serialPort.c
    ../../src/serialPortPlatform.c
)

if (WIN32)
    target_link_libraries(ISAsciiExample)
else()
    target_link_libraries(ISAsciiExample m)
endif()

endif()
