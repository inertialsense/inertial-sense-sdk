add_library(InertialSense
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
  src/InertialSense.cpp
  src/inertialSenseBootLoader.c
  src/ISDataMappings.cpp
  src/ISDisplay.cpp
  src/ISLogger.cpp
  src/ISMatrix.c
  src/ISPose.c
  src/ISTcpClient.cpp
  src/ISUtilities.cpp
  src/linked_list.c
  src/Rtcm3Reader.cpp
  src/serialPort.c
  src/serialPortPlatform.c
  src/time_conversion.c
  src/tinystr.cpp
  src/tinyxml.cpp
  src/tinyxmlerror.cpp
  src/tinyxmlparser.cpp
  src/UbloxReader.cpp
  )

add_definitions(
  -Wno-format-extra-args
  )

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions")
