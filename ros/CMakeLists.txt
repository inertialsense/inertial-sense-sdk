cmake_minimum_required(VERSION 3.10.0)
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

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX")

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
  INCLUDE_DIRS include
  LIBRARIES inertial_sense_ros
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
)

# We don't need the example projects
set(IGNORE_EXAMPLE_PROJECTS 1)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ../src #This line of CMakeList.txt stays in .external file to reference submodule
  ../src/libusb/libusb
  ../src/libusb/linux
  ../src/yaml-cpp
)

add_subdirectory(.. devel)

add_library(inertial_sense_ros
    src/ParamHelper.cpp
    src/RtkRover.cpp
    src/RtkBase.cpp
    src/inertial_sense_ros.cpp
)

target_link_libraries(inertial_sense_ros InertialSense ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} pthread)
target_include_directories(inertial_sense_ros PUBLIC include)
add_dependencies(inertial_sense_ros inertial_sense_ros_generate_messages_cpp)

add_executable(inertial_sense_node src/inertial_sense_node.cpp)
target_link_libraries(inertial_sense_node inertial_sense_ros ${catkin_LIBRARIES})

catkin_add_gtest(test_unit_tests test/test_basic_unit_tests.cpp)
target_link_libraries(test_unit_tests ${PROJECT_NAME})

catkin_add_gtest(test_ros_bridge test/test_main.cpp test/test_communications.cpp test/test_client_reconnect.cpp)
target_link_libraries(test_ros_bridge ${PROJECT_NAME})

# catkin_add_gtest(test_client_reconnect test/test_client_reconnect.cpp)
# target_link_libraries(test_client_reconnect ${PROJECT_NAME})
