cmake_minimum_required(VERSION 2.8.12)
project(pySDK)

add_subdirectory(pybind11-2.1.1)

set(RT_DIR "C:/Users/netadmin/Documents/InertialSense/Repos/IS-src/cpp/SDK")
#set(RT_DIR ${CMAKE_SOURCE_DIR})
set(PYTHON_DIR "C:/Python27")

set(SRC_DIR "${RT_DIR}/src")
set(PYBIND_DIR "${RT_DIR}/pySDK/pybind11-2.1.1")
set(OUT_DIR "${RT_DIR}/pySDK/src")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${OUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# Set a default build configuration if none is specified. 'MinSizeRel' produces the smallest binaries
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'MinSizeRel' as none was specified.")
  set(CMAKE_BUILD_TYPE MinSizeRel CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" U_CMAKE_BUILD_TYPE)

# Include path for Python header files
include_directories(${PYTHON_DIR}/include)

# Include path for pybind11 header files -- this may need to be changed depending on your setup
include_directories(${PYBIND_DIR}/include)

add_definitions(-DENABLE_IS_PYTHON_WRAPPER)

add_library(pySDK SHARED
  ${SRC_DIR}/com_manager.c
  ${SRC_DIR}/data_sets.c
  ${SRC_DIR}/DataChunk.cpp
  ${SRC_DIR}/DataChunkSorted.cpp
  ${SRC_DIR}/DataCSV.cpp
  ${SRC_DIR}/DataKML.cpp
  ${SRC_DIR}/DeviceLog.cpp
  ${SRC_DIR}/DeviceLogCSV.cpp
  ${SRC_DIR}/DeviceLogKML.cpp
  ${SRC_DIR}/DeviceLogSerial.cpp
  ${SRC_DIR}/DeviceLogSorted.cpp
  ${SRC_DIR}/GpsParser.cpp
  ${SRC_DIR}/InertialSense.cpp
  ${SRC_DIR}/inertialSenseBootLoader.c
  ${SRC_DIR}/ISDataMappings.cpp
  ${SRC_DIR}/ISDisplay.cpp
  ${SRC_DIR}/ISLogger.cpp
  ${SRC_DIR}/ISMatrix.c
  ${SRC_DIR}/ISPose.c
  ${SRC_DIR}/ISTcpClient.cpp
  ${SRC_DIR}/ISTcpServer.cpp
  ${SRC_DIR}/ISUtilities.cpp
  ${SRC_DIR}/linked_list.c
  ${SRC_DIR}/serialPort.c
  ${SRC_DIR}/serialPortPlatform.c
  ${SRC_DIR}/time_conversion.c
  ${SRC_DIR}/tinystr.cpp
  ${SRC_DIR}/tinyxml.cpp
  ${SRC_DIR}/tinyxmlerror.cpp
  ${SRC_DIR}/tinyxmlparser.cpp
  ${SRC_DIR}/../pySDK/pySDK.cpp
  )

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions")

# Don't add a 'lib' prefix to the shared library
#set_target_properties(pySDK PROPERTIES PREFIX "")

if (WIN32)
  add_definitions(-DNOMINMAX)
  if (MSVC)
    # /bigobj is needed for bigger binding projects due to the limit to 64k
    # addressable sections. /MP enables multithreaded builds (relevant when
    # there are many files).
    set_target_properties(pySDK PROPERTIES COMPILE_FLAGS "/MP /bigobj ")

    if (NOT ${U_CMAKE_BUILD_TYPE} MATCHES DEBUG)
      # Enforce size-based optimization and link time code generation on MSVC
      # (~30% smaller binaries in experiments).
      set_target_properties(pySDK APPEND_STRING PROPERTY COMPILE_FLAGS "/Os /GL ")
      set_target_properties(pySDK APPEND_STRING PROPERTY LINK_FLAGS "/LTCG ")
    endif()
  endif()

  # .PYD file extension on Windows
  set_target_properties(pySDK PROPERTIES SUFFIX ".pyd")

  # Link against the Python shared library
  target_link_libraries(pySDK "${PYTHON_DIR}/libs/python27.lib")
  target_link_libraries(pySDK wsock32 ws2_32)
elseif (UNIX)
  # It's quite common to have multiple copies of the same Python version
  # installed on one's system. E.g.: one copy from the OS and another copy
  # that's statically linked into an application like Blender or Maya.
  # If we link our plugin library against the OS Python here and import it
  # into Blender or Maya later on, this will cause segfaults when multiple
  # conflicting Python instances are active at the same time (even when they
  # are of the same version).

  # Windows is not affected by this issue since it handles DLL imports
  # differently. The solution for Linux and Mac OS is simple: we just don't
  # link against the Python library. The resulting shared library will have
  # missing symbols, but that's perfectly fine -- they will be resolved at
  # import time.

  # .SO file extension on Linux/Mac OS
  set_target_properties(pySDK PROPERTIES SUFFIX ".so")

  # Strip unnecessary sections of the binary on Linux/Mac OS
  if(APPLE)
    set_target_properties(pySDK PROPERTIES MACOSX_RPATH ".")
    set_target_properties(pySDK PROPERTIES LINK_FLAGS "-undefined dynamic_lookup ")
    if (NOT ${U_CMAKE_BUILD_TYPE} MATCHES DEBUG)
      add_custom_command(TARGET pySDK POST_BUILD COMMAND strip -u -r ${PROJECT_BINARY_DIR}/pySDK.so)
    endif()
  else()
    if (NOT ${U_CMAKE_BUILD_TYPE} MATCHES DEBUG)
      add_custom_command(TARGET pySDK POST_BUILD COMMAND strip ${PROJECT_BINARY_DIR}/pySDK.so)
    endif()
  endif()
endif()