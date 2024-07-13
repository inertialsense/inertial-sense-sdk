CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

project(IS-SDK_unit-tests)

set(IS_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/..")

find_library(SDK_LIBRARY_PATH InertialSenseSDK PATHS ${IS_SDK_DIR})
if(NOT SDK_LIBRARY_PATH AND NOT TARGET InertialSenseSDK)
    # InertialSenseSDK library not prebuilt and not already include in project.  Include in this project.
    add_subdirectory(${IS_SDK_DIR} InertialSenseSDK)
endif()

# Include InertialSenseSDK header files
include_directories(
    ${IS_SDK_DIR}/src
    ${IS_SDK_DIR}/src/protocol
    ${IS_SDK_DIR}/src/libusb/libusb
    ${IS_SDK_DIR}/src/util
    ${IS_SDK_DIR}/tests/runtime
)

find_package(GTest REQUIRED)

# Link the InertialSenseSDK static library 
link_directories(${IS_SDK_DIR})

file(GLOB TESTS_SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_*.cpp")
list(FILTER TESTS_SOURCES EXCLUDE REGEX "test_com_manager_2.cpp")
if(WIN32)   # These do not run in Windows
    list(FILTER TESTS_SOURCES EXCLUDE REGEX "test_time_conversion.cpp")
endif()

# Define the executable
add_executable(${PROJECT_NAME} 
    ${TESTS_SOURCES}
    runtime/device_runtime_tests.cpp

    # test_ISLogger.cpp
    # test_protocol_nmea.cpp
    # test_time_conversion.cpp
    
    # test_data_utils.cpp
    # test_runtime_tests.cpp
)

# Link InertialSenseSDK static library to the executable
target_link_libraries(${PROJECT_NAME} gtest_main InertialSenseSDK ${GTEST_LIBRARIES} udev pthread m)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 17)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

