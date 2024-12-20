CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)
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
    # test_data_utils.cpp

    # test_protocol_nmea.cpp
    # test_time_conversion.cpp    
    # test_runtime_tests.cpp
    # test_data_utils.cpp
)

if(WIN32)   # Windows
    # Check if VCPKG_ROOT is defined
    if(NOT DEFINED ENV{VCPKG_ROOT})
        message(FATAL_ERROR "The environment variable VCPKG_ROOT is not defined. Please run install_buiild_tools.bat or set it to the path of your vcpkg installation.")
    endif()

    # Use the VCPKG_ROOT system variable
    set(VCPKG_PATH $ENV{VCPKG_ROOT})
    set(GTEST_LIBRARY "${VCPKG_PATH}/installed/x64-windows/lib/gtest.lib")
    set(GTEST_MAIN_LIBRARY "${VCPKG_PATH}/installed/x64-windows/lib/manual-link/gtest_main.lib")
    target_link_directories(${PROJECT_NAME} PRIVATE "${VCPKG_PATH}/installed/x64-windows/lib/manual-link")
    set(PROJECT_PLATFORM_LIBS ${GTEST_LIBRARY} ${GTEST_MAIN_LIBRARY} GTest::GTest GTest::Main)
else()      # Linux
    set(PROJECT_PLATFORM_LIBS gtest_main ${GTEST_LIBRARIES} udev pthread m)
endif()

# Link InertialSenseSDK static library to the executable
target_link_libraries(${PROJECT_NAME} ${PROJECT_PLATFORM_LIBS} InertialSenseSDK)
