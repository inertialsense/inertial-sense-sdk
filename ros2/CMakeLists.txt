set(CMAKE_CXX_STANDARD 17)
cmake_minimum_required(VERSION 3.10.0)
project(inertial_sense_ros2)
#find_package(rosidl_typesupport_c REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Threads)

#set(RELATIVE_SCRIPT_PATH "${CMAKE_CURRENT_LIST_DIR}/../inertial-sense-sdk/scripts/build_is_sdk.sh")
#get_filename_component(ABSOLUTE_SCRIPT_PATH ${RELATIVE_SCRIPT_PATH} ABSOLUTE BASE_DIR ${CMAKE_SOURCE_DIR})
#add_custom_target(
#	RunScript
#	COMMAND ${CMAKE_COMMAND} -E env bash ${ABSOLUTE_SCRIPT_PATH}
#    COMMENT "Running custom shell script..."
#    VERBATIM
#	)


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
        DEPENDENCIES std_msgs geometry_msgs diagnostic_msgs builtin_interfaces
)
ament_export_include_directories(include)
ament_export_libraries(inertial_sense_ros2)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs std_msgs diagnostic_msgs)

# We don't need the example projects
set(IGNORE_EXAMPLE_PROJECTS 1)

include_directories(include
#<<<<<<< Updated upstream
  ${XMLRPC_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ../src #This line of CMakeList.txt stays in .external file to reference submodule
  ../src/libusb/libusb
  ../src/libusb/linux
  ../src/yaml-cpp
  ../ros2/build/inertial_sense_ros2/rosidl_generator_cpp/
  /opt/ros/jazzy/include
  /opt/ros/jazzy/include/sensor_msgs
)

# Build using C11 and CXX20
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX")


add_executable(new_target
        src/inertial_sense_ros2.cpp
        src/inertial_sense_node.cpp
        src/ParamHelper.cpp
        # src/serial.cpp
        include/TopicHelper.cpp
        src/RtkRover.cpp
        src/RtkBase.cpp
        #include/TopicHelper.h
)

ament_add_gtest(test_new_target
        src/inertial_sense_ros2.cpp
        src/ParamHelper.cpp
        include/TopicHelper.cpp
        src/RtkRover.cpp
        src/RtkBase.cpp
        #test/test_basic_unit_tests.cpp
        test/test_client_reconnect.cpp
        test/test_communications.cpp
        test/test_main.cpp
        test/test_main.h
        test/gtest_helpers.h
)
#add_dependencies(new_target RunScript)
rosidl_target_interfaces(new_target ${PROJECT_NAME} rosidl_typesupport_cpp)  # Ensure this matches your package and type )
ament_target_dependencies(new_target rclcpp std_msgs geometry_msgs nav_msgs diagnostic_msgs sensor_msgs std_srvs)

rosidl_target_interfaces(test_new_target ${PROJECT_NAME} rosidl_typesupport_cpp)  # Ensure this matches your package and type )
ament_target_dependencies(test_new_target rclcpp std_msgs geometry_msgs nav_msgs diagnostic_msgs sensor_msgs std_srvs)

get_filename_component(RESOLVED_ROS_DIR ${CMAKE_SOURCE_DIR} REALPATH) # Symlink hell

target_link_libraries(new_target ${RESOLVED_ROS_DIR}/../libInertialSenseSDK.a ${YAML_CPP_LIBRARIES} pthread)
target_link_libraries(test_new_target ${RESOLVED_ROS_DIR}/../libInertialSenseSDK.a ${YAML_CPP_LIBRARIES} pthread)


#link_directories(/home/s/Inertial_Sense/imx/ros2_ws/src/inertial-sense-sdk)
install(TARGETS new_target test_new_target
        DESTINATION lib/${PROJECT_NAME})
ament_package()
