cmake_minimum_required(VERSION 2.8.12)
project(log_reader)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
set(IGNORE_EXAMPLE_PROJECTS 1)

# if(INCLUDE_LUNA_DATA_SETS)	# run "cmake -DINCLUDE_LUNA_DATA_SETS=1 .."
add_definitions(-DINCLUDE_LUNA_DATA_SETS)
# endif()

find_package(pybind11 REQUIRED)
find_package(Threads)

add_definitions(-DPYBIND11_PYTHON_VERSION=3.6.6)

set(SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../)
add_subdirectory(${SDK_DIR} SDK)

include_directories(
    include
    ${SDK_DIR}/src
)

pybind11_add_module(log_reader
    src/log_reader.cpp
    include/log_reader.h
)

if(WIN32)
	# Link WinSock and InertialSense library
	target_link_libraries(${PROJECT_NAME} PRIVATE Ws2_32.lib InertialSense)
else()
	# Link just InertialSense library
	target_link_libraries(${PROJECT_NAME} PRIVATE InertialSense)
endif()

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(log_reader PROPERTIES
    PREFIX ""
    OUTPUT_NAME "log_reader"
    SUFFIX ".so"
    LIBRARY_OUTPUT_DIRECTORY ..
)
