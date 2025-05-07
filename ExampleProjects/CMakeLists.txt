CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

project(ExampleProjects)

if (MSVC)
    # Use modern standards with MSVC, scoped per language
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    set(CMAKE_C_EXTENSIONS OFF)
endif()

# Add InertialSenseSDK static library subdirectory
set(IS_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
include(${IS_SDK_DIR}/include_is_sdk_find_library.cmake)

if(WIN32)
    # yamlcpp needs to know it's a static lib
    add_definitions(-DYAML_CPP_STATIC_DEFINE)
endif()

# Add all the other sub projects
add_subdirectory(Ascii)
add_subdirectory(Bootloader)
add_subdirectory(ISComm)
add_subdirectory(ISComm_callback)
add_subdirectory(IS_firmwareUpdate_v2)
# add_subdirectory(InertialSense_log_reader)
add_subdirectory(InertialSense_log_reader_RegCmp)
add_subdirectory(ISLogger_read)
add_subdirectory(ISLogger_write)
add_subdirectory(InertialSense_logger)
add_subdirectory(NTRIP_rover)
add_subdirectory(IS_NMEAProtocolCheckSum)
