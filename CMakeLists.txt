add_library(InertialSense
  src/cltool.cpp
  src/cltool_main.cpp
  src/com_manager.c
  src/data_sets.c
  src/DataChunk.cpp
  src/DataChunkSorted.cpp
  src/DataCSV.cpp
  src/DataKML.cpp
  src/DeviceLog.cpp
  src/DeviceLogCSV.cpp
  src/DeviceLogKML.cpp
  src/DeviceLogSerial.cpp
  src/DeviceLogSorted.cpp
  src/GpsParser.cpp
  src/InertialSense.cpp
  src/inertialSenseBootLoader.c
  src/ISDataMappings.cpp
  src/ISDisplay.cpp
  src/ISEarth.c
  src/ISLogger.cpp
  src/ISMatrix.c
  src/ISPose.c
  src/ISSerialPort.cpp
  src/ISStream.cpp
  src/ISTcpClient.cpp
  src/ISTcpServer.cpp
  src/ISUtilities.cpp
  src/linked_list.c
  src/serialPort.c
  src/serialPortPlatform.c
  src/time_conversion.c
  src/tinystr.cpp
  src/tinyxml.cpp
  src/tinyxmlerror.cpp
  src/tinyxmlparser.cpp
  )

add_definitions(
  -Wno-format-extra-args
  )

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions")