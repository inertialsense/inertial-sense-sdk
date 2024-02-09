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

find_library(SDK_LIBRARY_PATH InertialSenseSDK PATHS ${IS_SDK_DIR})
if(NOT SDK_LIBRARY_PATH)
    # InertialSenseSDK static library not built.  Build SDK along with this project. 
    add_subdirectory(${IS_SDK_DIR} InertialSenseSDK)
endif()

# Add all the other sub projects
add_subdirectory(Ascii)
add_subdirectory(Bootloader)
add_subdirectory(Communications)
add_subdirectory(IS_firmwareUpdate_v2)
add_subdirectory(Logger)
add_subdirectory(LogReader)
add_subdirectory(NTRIP_rover)
add_subdirectory(IS_NMEAProtocolCheckSum)
