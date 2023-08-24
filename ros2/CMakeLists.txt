cmake_minimum_required(VERSION 3.8)
project(inertial_sense_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/DIDINS1.msg
  msg/DIDINS2.msg
  msg/DIDINS4.msg
  msg/GlonassEphemeris.msg
  msg/GNSSEphemeris.msg
  msg/GNSSObservation.msg
  msg/GNSSObsVec.msg
  msg/GPS.msg
  msg/GPSInfo.msg
  msg/GTime.msg
  msg/INL2States.msg
  msg/PIMU.msg
  msg/RTKInfo.msg
  msg/RTKRel.msg
  msg/SatInfo.msg
  srv/FirmwareUpdate.srv
  srv/RefLLAUpdate.srv
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(include
  ${YAML_CPP_INCLUDE_DIR}
  ../src
  ../src/libusb/libusb
  ../src/libusb/linux
  ../src/yaml-cpp
)

set(IGNORE_EXAMPLE_PROJECTS ON)
add_subdirectory(.. build)

add_library(inertial_sense_ros_lib
    src/ParamHelper.cpp
    src/RtkRover.cpp
    src/RtkBase.cpp
    src/inertial_sense_ros.cpp
)
target_link_libraries(inertial_sense_ros_lib InertialSense ${YAML_CPP_LIBRARIES} ${cpp_typesupport_target} pthread)
ament_target_dependencies(inertial_sense_ros_lib rclcpp rosgraph_msgs std_msgs std_srvs geometry_msgs sensor_msgs nav_msgs diagnostic_msgs tf2_ros tf2_geometry_msgs)

add_executable(inertial_sense_node src/inertial_sense_node.cpp)
target_link_libraries(inertial_sense_node inertial_sense_ros_lib) # ${ament_LIBRARIES}
ament_target_dependencies(inertial_sense_node rclcpp rosgraph_msgs std_msgs std_srvs geometry_msgs sensor_msgs nav_msgs diagnostic_msgs tf2_ros tf2_geometry_msgs)

ament_export_dependencies(
  # rosidl_default_runtime
  std_msgs
  rclcpp
  sensor_msgs
  geometry_msgs
  rosgraph_msgs
  diagnostic_msgs
  tf2_ros
  tf2_geometry_msgs
)
ament_export_include_directories(include)
ament_export_libraries(inertial_sense_ros_lib)

install(TARGETS inertial_sense_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS inertial_sense_ros_lib LIBRARY DESTINATION lib)
install(TARGETS InertialSense LIBRARY DESTINATION lib)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
