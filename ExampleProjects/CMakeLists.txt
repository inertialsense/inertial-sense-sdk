CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

project(ExampleProjects)

if (MSVC_VERSION GREATER_EQUAL "1900")
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("/std:c++latest" _cpp_latest_flag_supported)
    if (_cpp_latest_flag_supported)
        add_compile_options("/std:c++latest")
    endif()
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
add_subdirectory(Communications)
add_subdirectory(IS_firmwareUpdate_v2)
# add_subdirectory(InertialSense_log_reader)
add_subdirectory(InertialSense_log_reader_RegCmp)
add_subdirectory(ISLogger_read)
add_subdirectory(ISLogger_write)
add_subdirectory(InertialSense_logger)
add_subdirectory(NTRIP_rover)
add_subdirectory(IS_NMEAProtocolCheckSum)
