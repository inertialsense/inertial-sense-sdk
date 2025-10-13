CMAKE_MINIMUM_REQUIRED(VERSION 3.21.0)
if(WIN32)
set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")	# Must go before calling project()
endif()

project(IS-SDK_unit-tests)

add_definitions(-DSDK_UNIT_TEST)

set(IS_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
include(${IS_SDK_DIR}/include_is_sdk_find_library.cmake)

# Include InertialSenseSDK header files
include_directories(
    ${IS_SDK_DIR}/src
    ${IS_SDK_DIR}/src/core
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
    # yamlcpp needs to know it's a static lib
    add_definitions(-DYAML_CPP_STATIC_DEFINE)
endif()

# Define the executable
add_executable(${PROJECT_NAME} 
    ${TESTS_SOURCES}
    runtime/device_runtime_tests.cpp

    # test_data_utils.cpp
    # test_serial_utils.cpp

    # test_ISLogger.cpp
    # test_protocol_nmea.cpp
    # test_time_conversion.cpp    
    # test_runtime_tests.cpp
    # test_ISDataMappings.cpp
)

if(WIN32)   # Windows
    find_package(GTest CONFIG REQUIRED)  # <— note CONFIG

    if (WIN32)
    # Don't hardcode library paths; use vcpkg’s imported targets
    set(PROJECT_PLATFORM_LIBS
        GTest::gtest
        GTest::gtest_main
        ws2_32
    )
    else()
    set(PROJECT_PLATFORM_LIBS
        GTest::gtest
        GTest::gtest_main
        pthread
        m
    )
    endif()
else()      # Linux
    set(PROJECT_PLATFORM_LIBS gtest_main ${GTEST_LIBRARIES} udev pthread m)
endif()

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 17)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

# Link IS-SDK libraries to the executable
include(${IS_SDK_DIR}/include_is_sdk_target_link_libraries.cmake)

# Link InertialSenseSDK static library to the executable
target_link_libraries(${PROJECT_NAME} ${PROJECT_PLATFORM_LIBS})