add_library(InertialSense
  src/InertialSense.cpp
  src/inertialSenseBootLoader.c
  
  src/com_manager.c
  src/data_sets.c
  src/linked_list.c
  
  src/ISLogger.cpp
  src/DeviceLog.cpp
  src/DeviceLogSerial.cpp
  src/DeviceLogSorted.cpp
  src/DeviceLogCSV.cpp
  src/DeviceLogKML.cpp
  src/DataChunk.cpp
  src/DataChunkSorted.cpp
  src/DataCSV.cpp
  src/DataKML.cpp
  
  src/ISMatrix.c
  src/ISPose.c
  
  src/serialPort.c
  src/serialPortPlatform.c
  
  src/ISUtilities.cpp
  src/ISDisplay.cpp
  src/ISDataMappings.cpp
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
