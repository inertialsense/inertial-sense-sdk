cmake_minimum_required(VERSION 3.10.0)
add_definitions(-DROS1)
project(inertial_sense_ros)

find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  message_generation
  tf
)
find_package(Threads)

add_message_files(
  FILES
  GTime.msg
  SatInfo.msg
  GPS.msg
  GPSInfo.msg
  PIMU.msg
  RTKInfo.msg
  RTKRel.msg
  GlonassEphemeris.msg
  GNSSEphemeris.msg
  GNSSObservation.msg
  GNSSObsVec.msg
  INL2States.msg
  DID_INS2.msg
  DID_INS1.msg
  DID_INS4.msg
)

add_service_files(
  FILES
  FirmwareUpdate.srv
  refLLAUpdate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS shared/include
  LIBRARIES inertial_sense_ros
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
)

# We don't need the example projects
set(IGNORE_EXAMPLE_PROJECTS 1)

include_directories(
  shared/include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ../../src #This line of CMakeList.txt stays in .external file to reference submodule
  ../../src/libusb/libusb
  ../../src/libusb/linux
  ../../src/yaml-cpp
)

add_subdirectory(../.. devel)

add_library(inertial_sense_ros
    shared/src/ParamHelper.cpp
    shared/src/RtkRover.cpp
    shared/src/RtkBase.cpp
    src/inertial_sense_ros.cpp
)

# Build using C11 and CXX20
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX")

target_link_libraries(inertial_sense_ros InertialSenseSDK ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} pthread)
target_include_directories(inertial_sense_ros PUBLIC shared/include)
add_dependencies(inertial_sense_ros inertial_sense_ros_generate_messages_cpp)

add_executable(inertial_sense_node shared/src/inertial_sense_node.cpp)
target_link_libraries(inertial_sense_node inertial_sense_ros ${catkin_LIBRARIES})

catkin_add_gtest(test_unit_tests test/test_basic_unit_tests.cpp)
target_link_libraries(test_unit_tests ${PROJECT_NAME})

catkin_add_gtest(test_ros_bridge test/test_main.cpp test/test_communications.cpp test/test_client_reconnect.cpp)
target_link_libraries(test_ros_bridge ${PROJECT_NAME})

# catkin_add_gtest(test_client_reconnect test/test_client_reconnect.cpp)
# target_link_libraries(test_client_reconnect ${PROJECT_NAME})
