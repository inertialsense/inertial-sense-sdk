CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)

project(ISCommunicationsExample)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#FIND_PACKAGE(Threads)

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

if(APPLE)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
else()
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions -Wl,--no-as-needed")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions -Wl,--no-as-needed")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lm")
endif()

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_executable(ISCommunicationsExample
	"ISCommunicationsExample.c"
	"../../src/data_sets.c"
	"../../src/ISComm.c"
	"../../src/serialPort.c"
	"../../src/serialPortPlatform.c"
)

