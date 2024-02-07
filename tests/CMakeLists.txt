CMAKE_MINIMUM_REQUIRED(VERSION 3.10.0)

project(IS-SDK_unit-tests)

set(IS_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/..")

# Add InertialSenseSDK static library subdirectory
add_subdirectory(${IS_SDK_DIR} InertialSenseSDK)

# Include InertialSenseSDK header files
include_directories(
    ${IS_SDK_DIR}/src
    ${IS_SDK_DIR}/src/libusb/libusb
)

find_package(GTest REQUIRED)

# Link the InertialSenseSDK static library 
link_directories(${IS_SDK_DIR})

file(GLOB TESTS_SOURCES "${CMAKE_CURRENT_LIST_DIR}/test_*.cpp")
list(FILTER TESTS_SOURCES EXCLUDE REGEX "test_com_manager_2.cpp")

# Define the executable
add_executable(${PROJECT_NAME} ${TESTS_SOURCES})

# Link InertialSenseSDK static library to the executable
# target_link_libraries(${PROJECT_NAME} InertialSenseSDK)
target_link_libraries(${PROJECT_NAME} gtest_main InertialSenseSDK ${GTEST_LIBRARIES} udev pthread m)







# Required: set SOURCES_PROJECT to the code in this project, the files must go on their own lines
# set(SOURCES_PROJECT ${TESTS_SOURCES})

# include_directories(${GTEST_INCLUDE_DIRS}
# 		${PROJECT_DIR}/src
# 		${PROJECT_DIR}/src/libusb
# 		${PROJECT_DIR}/src/libusb/linux
# 		${PROJECT_DIR}/src/libusb/libusb
# 		${PROJECT_DIR}/src/yaml-cpp
# 		${PROJECT_DIR}/external
# 		)


# add_executable(${PROJECT_NAME} ${SOURCES_PROJECT})

# SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions")
