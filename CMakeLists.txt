CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12)

project(InertialSense)

if (!WIN32)
  #SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -Wall -Wextra -Wno-unused-function -Wno-unused-value -Wno-unused-parameter -Wno-missing-field-initializers")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Wextra -Wno-unused-function -Wno-unused-value -Wno-unused-parameter -Wno-missing-field-initializers")
endif()

SET(BUILD_INERTIAL_SENSE_SDK 1)

add_definitions(-DRTK_EMBEDDED)
find_package(Threads REQUIRED)
if (!WIN32)
  find_package(yaml-cpp REQUIRED)
endif()

FILE(GLOB SOURCES_ALL
  src/*.c
  src/*.cpp
  src/*.h
)

add_library(InertialSense ${SOURCES_ALL})

find_library(LIBUSB_LIBRARY
	NAMES usb-1.0
	PATH_SUFFIXES "lib" "lib32" "lib64"
)

if(WIN32)
	add_definitions(-DYAML_CPP_STATIC_DEFINE)
	target_include_directories(InertialSense PUBLIC
		src/libusb/msvc
	)
else()
	target_include_directories(InertialSense PUBLIC
		src/libusb/linux
	)
endif()

target_include_directories(InertialSense PUBLIC 
  src 
  external 
  src/libusb/libusb 
  src/libusb
)

if (WIN32)
	target_link_directories(InertialSense PRIVATE src/libs)
  target_link_libraries(InertialSense)
else()
  target_link_libraries(InertialSense pthread ${LIBUSB_LIBRARY} ${YAML_CPP_LIBRARIES} m yaml-cpp)
endif()

target_compile_options(InertialSense PUBLIC -fPIC)

add_library( InertialSense::libinertialsense ALIAS InertialSense )

add_subdirectory(cltool)
add_subdirectory(ExampleProjects)
