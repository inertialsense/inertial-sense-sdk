set(CMAKE_CXX_STANDARD 17)
cmake_minimum_required(VERSION 3.10.0)
project(inertial_sense_ros2)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(Threads)
set_property(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS 1)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/GTime.msg"
        "msg/SatInfo.msg"
        "msg/GPS.msg"
        "msg/GPSInfo.msg"
        "msg/PIMU.msg"
        "msg/RTKInfo.msg"
        "msg/RTKRel.msg"
        "msg/GlonassEphemeris.msg"
        "msg/GNSSEphemeris.msg"
        "msg/GNSSObservation.msg"
        "msg/GNSSObsVec.msg"
        "msg/INL2States.msg"
        "msg/DIDINS2.msg"
        "msg/DIDINS1.msg"
        "msg/DIDINS4.msg"
        "srv/FirmwareUpdate.srv"
        "srv/RefLLAUpdate.srv"
        DEPENDENCIES std_msgs geometry_msgs
)
ament_export_include_directories(include)
ament_export_libraries(inertial_sense_ros2)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs)

# We don't need the example projects
set(IGNORE_EXAMPLE_PROJECTS 1)

include_directories(include
  #${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ../src #This line of CMakeList.txt stays in .external file to reference submodule
  ../src/libusb/libusb
  ../src/libusb/linux
  ../src/yaml-cpp
        /home/s/Inertial_Sense/imx/ros2_ws/src/inertial-sense-sdk/ros2/build/inertial_sense_ros2/rosidl_generator_cpp/inertial_sense_ros2
  /opt/ros/jazzy/include
  /opt/ros/jazzy/include/sensor_msgs
)

#add_library(inertial_sense_ros2
#    src/ParamHelper.cpp
#    src/RtkRover.cpp
#    src/RtkBase.cpp
#    src/inertial_sense_ros.cpp
#)

# Build using C11 and CXX20
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX")

#target_link_libraries(inertial_sense_ros2 InertialSenseSDK ${YAML_CPP_LIBRARIES} pthread)
#target_include_directories(inertial_sense_ros2 PUBLIC include)
#add_dependencies(inertial_sense_ros2 inertial_sense_ros_generate_messages_cpp)
#
#add_executable(inertial_sense_node src/inertial_sense_node.cpp)
#target_link_libraries(inertial_sense_node inertial_sense_ros2)
#
#ament_add_gtest(test_unit_tests test/test_basic_unit_tests.cpp)
#target_link_libraries(test_unit_tests ${PROJECT_NAME})
#
#ament_add_gtest(test_ros_bridge test/test_main.cpp test/test_communications.cpp test/test_client_reconnect.cpp)
#target_link_libraries(test_ros_bridge ${PROJECT_NAME})
#
# catkin_add_gtest(test_client_reconnect test/test_client_reconnect.cpp)
# target_link_libraries(test_client_reconnect ${PROJECT_NAME})

add_executable(new_target src/inertial_sense_ros.cpp
        src/inertial_sense_node.cpp
        src/ParamHelper.cpp
        src/serial.cpp
        include/TopicHelper.cpp
)
