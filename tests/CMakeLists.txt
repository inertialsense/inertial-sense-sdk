CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

# Required: set the project
project(IS-SDK_unit-tests)

# Optional: add any libraries this project needs
#set(PROJECT_LIBS "")

find_package(GTest REQUIRED)

file(GLOB TESTS_SRC "${CMAKE_CURRENT_LIST_DIR}/test_*.cpp")
list(FILTER TESTS_SRC EXCLUDE REGEX "test_com_manager_2.cpp")

# Required: set SOURCES_PROJECT to the code in this project, the files must go on their own lines
set(SOURCES_PROJECT ${TESTS_SRC})

include_directories(${GTEST_INCLUDE_DIRS}
		${PROJECT_DIR}/src
		${PROJECT_DIR}/src/libusb
		${PROJECT_DIR}/src/libusb/linux
		${PROJECT_DIR}/src/libusb/libusb
		${PROJECT_DIR}/src/yaml-cpp
		${PROJECT_DIR}/external
		)

# Everything is setup, use common template to build
include(../CMakeCommon.txt)

#add_library(${PROJECT_NAME}
#		${TESTS_SRC}
#		${SOURCES_SDK}
#		)

# add_executable(${PROJECT_NAME} ${SOURCES_PROJECT})
# add_executable(${PROJECT_NAME} ${TESTS_SRC})

# add_executable(${PROJECT_NAME} ${SOURCES_PROJECT})
target_link_libraries(${PROJECT_NAME} gtest_main ${GTEST_LIBRARIES} udev pthread m)

#    target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
#    add_test(NAME ${PROJECT_NAME}
#             COMMAND ${PROJECT_NAME} --gtest_color=true)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions")
