CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

project(ISFirmwareUpdateExample_v2)

set(IS_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../..")
include(${IS_SDK_DIR}/include_is_sdk_find_library.cmake)

# Include InertialSenseSDK header files
include_directories(
    ${IS_SDK_DIR}/src
    ${IS_SDK_DIR}/src/libusb/libusb
)

# Link the InertialSenseSDK static library 
link_directories(${IS_SDK_DIR})

# Define the executable
add_executable(${PROJECT_NAME} ISFirmwareUpdateExample_v2.cpp)

# Link IS-SDK libraries to the executable
include(${IS_SDK_DIR}/include_is_sdk_target_link_libraries.cmake)
