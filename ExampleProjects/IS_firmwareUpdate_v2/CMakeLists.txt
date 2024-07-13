CMAKE_MINIMUM_REQUIRED(VERSION 3.21.0)

project(ISFirmwareUpdateExample_v2)

set(IS_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../..")

find_library(SDK_LIBRARY_PATH InertialSenseSDK PATHS ${IS_SDK_DIR})
if(NOT SDK_LIBRARY_PATH AND NOT TARGET InertialSenseSDK)
    # InertialSenseSDK library not prebuilt and not already include in project.  Include in this project.
    add_subdirectory(${IS_SDK_DIR} InertialSenseSDK)
endif()

# Include InertialSenseSDK header files
include_directories(
    ${IS_SDK_DIR}/src
    ${IS_SDK_DIR}/src/libusb/libusb
)

# Link the InertialSenseSDK static library 
link_directories(${IS_SDK_DIR})

# Define the executable
add_executable(${PROJECT_NAME} ISFirmwareUpdateExample_v2.cpp)

# Find pthread package
find_package(Threads REQUIRED)

# Link InertialSenseSDK static library to the executable
target_link_libraries(${PROJECT_NAME} InertialSenseSDK Threads::Threads)

