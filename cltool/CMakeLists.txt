CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

# Specify the C and C++ compilers
# set(CMAKE_SYSROOT           /opt/intellian-ti-toolchain/armv7at2hf-neon-linux-gnueabi)
# set(CMAKE_FIND_ROOT_PATH    /opt/intellian-ti-toolchain/armv7at2hf-neon-linux-gnueabi)
# set(CMAKE_C_COMPILER        /opt/intellian-ti-toolchain/gcc-arm-9.2-2019.12-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-gcc)
# set(CMAKE_CXX_COMPILER      /opt/intellian-ti-toolchain/gcc-arm-9.2-2019.12-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-g++)

project(cltool)

set(IS_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
# find_library(SDK_LIBRARY_PATH InertialSenseSDK PATHS ${IS_SDK_DIR} ${IS_SDK_DIR}/build ${IS_SDK_DIR}/build/Release)
find_library(SDK_LIBRARY_PATH InertialSenseSDK PATHS ${IS_SDK_DIR})
if(NOT SDK_LIBRARY_PATH AND NOT TARGET InertialSenseSDK)
    # InertialSenseSDK library not prebuilt and not already include in project.  Include in this project.
    add_subdirectory(${IS_SDK_DIR} InertialSenseSDK)
else()
    link_directories(${IS_SDK_DIR}/Build/Release)
endif()

# Include InertialSenseSDK header files
include_directories(
    src
    ${IS_SDK_DIR}/src
    ${IS_SDK_DIR}/src/libusb/libusb
)

# Link the InertialSenseSDK static library 
link_directories(${IS_SDK_DIR})

if(WIN32)
    # yamlcpp needs to know it's a static lib
    add_definitions(-DYAML_CPP_STATIC_DEFINE)
endif()

# Define the executable
add_executable(${PROJECT_NAME} 
    src/cltool_main.cpp
    src/cltool.cpp
)

# Find pthread package
find_package(Threads REQUIRED)

# Link InertialSenseSDK static library to the executable
target_link_libraries(${PROJECT_NAME} InertialSenseSDK Threads::Threads)

