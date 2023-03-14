CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

project(inertial-sense-sdk)

#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -Wall -Wextra -Wno-unused-function -Wno-unused-value -Wno-unused-parameter -Wno-missing-field-initializers")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Wextra -Wno-unused-function -Wno-unused-value -Wno-unused-parameter -Wno-missing-field-initializers")


SET(BUILD_INERTIAL_SENSE_SDK 1)

add_definitions(-DRTK_EMBEDDED)
find_package(Threads REQUIRED)

add_library(InertialSense
  src/cltool.cpp
  src/cltool_main.cpp
  src/convert_ins.cpp
  src/com_manager.c
  src/data_sets.c
  src/DataChunk.cpp
  src/DataChunkSorted.cpp
  src/DataCSV.cpp
  src/DataJSON.cpp
  src/DataKML.cpp
  src/DeviceLog.cpp
  src/DeviceLogCSV.cpp
  src/DeviceLogJSON.cpp
  src/DeviceLogKML.cpp
  src/DeviceLogSerial.cpp
  src/DeviceLogSorted.cpp
  src/filters.cpp
  src/ihex.c
  src/InertialSense.cpp
  # src/ISBootloaderBase.cpp
  # src/ISBootloaderThread.cpp
	# src/ISBootloaderAPP.cpp
	# src/ISBootloaderDFU.cpp
	hw-libs/firmwareSignatures.c
	src/ISClient.cpp
  src/ISComm.c
  src/ISDataMappings.cpp
  src/ISDisplay.cpp
  src/ISEarth.c
  src/ISFileManager.cpp
  src/ISLogFile.cpp
  src/ISLogger.cpp
  src/ISLogStats.cpp
  src/ISMatrix.c
  src/ISPose.c
  src/ISSerialPort.cpp
  src/ISStream.cpp
  src/ISTcpClient.cpp
  src/ISTcpServer.cpp
  src/ISUtilities.cpp
	src/message_stats.cpp
  src/protocol_nmea.cpp
  src/linked_list.c
  src/ring_buffer.c
  src/serialPort.c
  src/serialPortPlatform.c
  src/statistics.cpp
  src/time_conversion.c
  src/tinystr.cpp
  src/tinyxml.cpp
  src/tinyxmlerror.cpp
  src/tinyxmlparser.cpp
)

find_library(LIBUSB_LIBRARY
	NAMES usb-1.0
	PATH_SUFFIXES "lib" "lib32" "lib64"
)

target_include_directories(InertialSense PUBLIC src external src/libusb/libusb src/libusb)
target_link_libraries(InertialSense pthread ${LIBUSB_LIBRARY} m)
target_compile_options(InertialSense PUBLIC -fPIC)

add_library( InertialSense::libinertialsense ALIAS InertialSense )

add_subdirectory(cltool)
add_subdirectory(ExampleProjects)
