CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)

project(ISNtripRoverExample)

#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers")

if(BUILD_INERTIAL_SENSE_SDK)	# Build w/ SDK

add_executable(ISNtripRoverExample ISNtripRoverExample.cpp)

else()	# Build only this project

add_executable(ISNtripRoverExample
	ISNtripRoverExample.cpp
	../../src/data_sets.c
	../../src/ISClient.cpp
	../../src/ISComm.c
	../../src/ISEarth.c
	../../src/ISFileManager.cpp
	../../src/ISMatrix.c
	../../src/ISPose.c
	../../src/ISSerialPort.cpp
	../../src/ISStream.cpp
	../../src/ISTcpClient.cpp
	../../src/ISTcpServer.cpp
	../../src/ISUtilities.cpp
	../../src/protocol_nmea.cpp
	../../src/serialPort.c
	../../src/serialPortPlatform.c
)
target_link_libraries(ISNtripRoverExample pthread)

endif()


