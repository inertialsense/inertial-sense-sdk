cmake_minimum_required(VERSION 2.8.12)
project(SDK_test)

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS} ../libusb/libusb)

add_library(SDK_test
	test_com_manager.cpp
	test_com_manager_2.cpp
	test_InertialSense.cpp
	test_ISComm.cpp
	test_ISDataMappings.cpp
	test_ISPolynomial.cpp
	test_math.cpp
	test_protocol_nmea.cpp
	test_ring_buffer.cpp
	test_time_conversion.cpp
	../com_manager.c
	../convert_ins.cpp
	../data_sets.c
	../DataChunk.cpp
	../DataChunkSorted.cpp
	../DataCSV.cpp
	../DataJSON.cpp
	../DataKML.cpp
	../DeviceLog.cpp
	../DeviceLogCSV.cpp
	../DeviceLogJSON.cpp
	../DeviceLogKML.cpp
	../DeviceLogSerial.cpp
	../DeviceLogSorted.cpp
	../ISComm.c
	../ISDataMappings.cpp
	../ISEarth.c
	../ISFileManager.cpp
	../ISLogFile.cpp
	../ISLogger.cpp
	../ISLogStats.cpp
	../ISMatrix.c
	../ISPolynomial.c
	../ISPose.c
	../ISUtilities.cpp
	../protocol_nmea.cpp
	../linked_list.c
	../ring_buffer.c
	../time_conversion.c
	../tinystr.cpp
	../tinyxml.cpp
	../tinyxmlerror.cpp
	../tinyxmlparser.cpp
	)

add_executable(run_tests 
	test_com_manager.cpp
	test_com_manager_2.cpp
	test_InertialSense.cpp
	test_ISComm.cpp
	test_ISDataMappings.cpp
	test_ISPolynomial.cpp
	test_math.cpp
	test_ring_buffer.cpp
	test_protocol_nmea.cpp
	test_time_conversion.cpp
	../com_manager.c
	../convert_ins.cpp
	../data_sets.c
	../ISComm.c
	../ISDataMappings.cpp
	../ISEarth.c
	../ISMatrix.c
	../ISPolynomial.c
	../ISPose.c
	../ISUtilities.cpp
	../protocol_nmea.cpp
	../linked_list.c
	../ring_buffer.c
	../time_conversion.c
	)

target_link_libraries(run_tests gtest_main ${GTEST_LIBRARIES} pthread)

#    target_include_directories(run_tests PUBLIC ${CMAKE_CURRENT_LIST_DIR})
#    add_test(NAME run_tests
#             COMMAND run_tests --gtest_color=true)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions")
