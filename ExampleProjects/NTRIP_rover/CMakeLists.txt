CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

project(ISNtripRoverExample)

set(IS_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../..")

if(NOT USE_PREBUILT_SDK)
    # Add InertialSenseSDK static library subdirectory to build SDK with this project 
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
add_executable(${PROJECT_NAME} ISNtripRoverExample.cpp)

# Link InertialSenseSDK static library to the executable
target_link_libraries(${PROJECT_NAME} InertialSenseSDK)