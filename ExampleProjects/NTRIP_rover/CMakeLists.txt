CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)

project(ISNtripRoverExample)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

if(BUILD_INERTIAL_SENSE_SDK)	# Build w/ SDK

add_executable(ISNtripRoverExample ISNtripRoverExample.cpp)
target_link_libraries(ISNtripRoverExample InertialSense)

else()	# Build only this project

add_executable(ISNtripRoverExample
    ISNtripRoverExample.cpp
    ../../src/data_sets.c
    ../../src/ISComm.c
    ../../src/ISMatrix.c
    ../../src/ISPose.c
	../../src/ISClient.cpp
	../../src/ISSerialPort.cpp
	../../src/ISStream.cpp
	../../src/ISTcpClient.cpp
	../../src/ISTcpServer.cpp
	../../src/ISUtilities.cpp
    ../../src/serialPort.c
    ../../src/serialPortPlatform.c
)
target_link_libraries(ISNtripRoverExample m)

endif()
