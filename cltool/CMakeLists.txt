CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

project(cltool)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(IS_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
include(${IS_SDK_DIR}/include_is_sdk_find_library.cmake)

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

# Link IS-SDK libraries to the executable
include(${IS_SDK_DIR}/include_is_sdk_target_link_libraries.cmake)
