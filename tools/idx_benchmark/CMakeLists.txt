CMAKE_MINIMUM_REQUIRED(VERSION 3.16.3)

project(idx_benchmark)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(IS_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../..")
include(${IS_SDK_DIR}/include_is_sdk_find_library.cmake)

# Include InertialSenseSDK header files + the tests/ dir for the
# GenerateRawLogData helper this tool reuses.
include_directories(
    src
    ${IS_SDK_DIR}/src
    ${IS_SDK_DIR}/src/core
    ${IS_SDK_DIR}/src/protocol
    ${IS_SDK_DIR}/src/util
    ${IS_SDK_DIR}/src/libusb/libusb
    ${IS_SDK_DIR}/tests
)

# Link the InertialSenseSDK static library
link_directories(${IS_SDK_DIR})

if(WIN32)
    # yamlcpp needs to know it's a static lib
    add_definitions(-DYAML_CPP_STATIC_DEFINE)
endif()

# Build the executable. We compile tests/test_data_utils.cpp directly
# into the binary because it provides GenerateRawLogData(), which is
# the synthetic-telemetry source the benchmark drives cISLogger with.
# That file lives in tests/ rather than src/ since it was originally
# written for unit tests; reusing it here keeps a single canonical
# "loggable test data" generator across both code paths.
add_executable(${PROJECT_NAME}
    src/idx_benchmark.cpp
    ${IS_SDK_DIR}/tests/test_data_utils.cpp
)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 17)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

# Link IS-SDK libraries to the executable
include(${IS_SDK_DIR}/include_is_sdk_target_link_libraries.cmake)

if(UNIX AND NOT APPLE)
    target_link_libraries(${PROJECT_NAME} pthread m)
endif()
