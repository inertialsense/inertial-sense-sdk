CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

project(ISLogReaderExample)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")


if(BUILD_INERTIAL_SENSE_SDK)	# Build w/ SDK

add_executable(ISLogReaderExample ISLogReaderExample.cpp)
target_link_libraries(ISLogReaderExample InertialSense)

else()	# Build only this project

add_executable(ISLogReaderExample
    ISLogReaderExample.cpp
	../../src/com_manager.c
	../../src/convert_ins.cpp
	../../src/data_sets.c
	../../src/DataChunk.cpp
	../../src/DataChunkSorted.cpp
	../../src/DataCSV.cpp
	../../src/DataJSON.cpp
	../../src/DataKML.cpp
	../../src/DeviceLog.cpp
	../../src/DeviceLogCSV.cpp
	../../src/DeviceLogJSON.cpp
	../../src/DeviceLogKML.cpp
	../../src/DeviceLogSerial.cpp
	../../src/DeviceLogSorted.cpp
	../../src/ihex.c
	../../src/InertialSense.cpp
	# ../../src/ISBootloaderBase.cpp
	# ../../src/ISBootloaderThread.cpp
	# ../../src/ISBootloaderAPP.cpp
	# ../../src/ISBootloaderDFU.cpp
	../../hw-libs/firmwareSignatures.c
	../../src/ISClient.cpp
	../../src/ISComm.c
	../../src/ISDataMappings.cpp
	../../src/ISDisplay.cpp
	../../src/ISEarth.c
	../../src/ISFileManager.cpp
	../../src/ISLogFile.cpp
	../../src/ISLogger.cpp
	../../src/ISLogStats.cpp
	../../src/ISMatrix.c
	../../src/ISPose.c
	../../src/ISSerialPort.cpp
	../../src/ISStream.cpp
	../../src/ISTcpClient.cpp
	../../src/ISTcpServer.cpp
	../../src/ISUtilities.cpp
	../../src/linked_list.c
	../../src/message_stats.cpp
	../../src/protocol_nmea.cpp
	../../src/serialPort.c
	../../src/serialPortPlatform.c
	../../src/time_conversion.c
	../../src/tinystr.cpp
	../../src/tinyxml.cpp
	../../src/tinyxmlerror.cpp
	../../src/tinyxmlparser.cpp    
)

find_library(LIBUSB_LIBRARY
	NAMES usb-1.0
	PATH_SUFFIXES "lib" "lib32" "lib64"
)

target_include_directories(ISLogReaderExample PUBLIC
	../../src
	../../src/libusb/libusb
	../../src/libusb/msvc
)

target_link_libraries(ISLogReaderExample pthread ${LIBUSB_LIBRARY} m)

endif()
