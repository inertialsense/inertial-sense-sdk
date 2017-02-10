CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)

project(InertialSenseCLTool)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

FIND_PACKAGE(Threads)

add_definitions(
  -O3
  -g
  -pedantic
#  -Wall        # Enable all warnings
  -Wextra
  -fvisibility=hidden
#  -DXPLM200
#  -DXPLM210
#  -DAPL=0
#  -DIBM=0
#  -DLIN=1
#  -fPIC
#  -DNDEBUG
  )

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions -Wl,--no-as-needed")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions -Wl,--no-as-needed")

SET(LIBPREFIX "${CMAKE_STATIC_LIBRARY_PREFIX}")
SET(LIBSUFFIX "${CMAKE_STATIC_LIBRARY_SUFFIX}")

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

ADD_SUBDIRECTORY(source)
