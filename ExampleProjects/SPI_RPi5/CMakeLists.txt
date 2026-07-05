CMAKE_MINIMUM_REQUIRED(VERSION 3.21.0)

project(SPIExample_RPi5)

# This example targets Linux only (Raspberry Pi 5 or any Linux SPI host).
# SPI support in the InertialSense SDK is implemented for Linux via sysfs/ioctl.
if(NOT UNIX OR APPLE)
    message(STATUS "SPIExample_RPi5: skipping (Linux only).")
    return()
endif()

set(IS_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../..")

# Find or include the InertialSense SDK.
# If a prebuilt libInertialSenseSDK.a is present in SDK/build/, it is used.
# Otherwise the SDK is built from source as a subdirectory.
include(${IS_SDK_DIR}/include_is_sdk_find_library.cmake)

# Include InertialSenseSDK header files
include_directories(
    ${IS_SDK_DIR}/src
    ${IS_SDK_DIR}/src/core
    ${IS_SDK_DIR}/src/libusb/libusb
)

# Link path for the SDK library
link_directories(${IS_SDK_DIR})

# Define the executable.
# spiPort.c contains the Linux sysfs/ioctl SPI port implementation.
# It is compiled here explicitly for the same reason serialPort.c is included
# in the ISComm example - platform-specific files may not be present in all
# prebuilt SDK library configurations.
add_executable(${PROJECT_NAME}
    SPIExample_RPi5.cpp
    ${IS_SDK_DIR}/src/core/spiPort.c
)

# Build using C17 and C++20 to match the SDK
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 17)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

# Link IS-SDK libraries to the executable
include(${IS_SDK_DIR}/include_is_sdk_target_link_libraries.cmake)
