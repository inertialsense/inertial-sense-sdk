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

add_definitions(-DROS2)

get_filename_component(RESOLVED_ROS_DIR ${CMAKE_SOURCE_DIR} REALPATH) # Symlink hell
message(RESOLVED_ROS_DIR="${RESOLVED_ROS_DIR}")

set(RELATIVE_SCRIPT_PATH "${CMAKE_CURRENT_LIST_DIR}/../inertial-sense-sdk/scripts/build_is_sdk.sh")
get_filename_component(ABSOLUTE_SCRIPT_PATH ${RELATIVE_SCRIPT_PATH} ABSOLUTE BASE_DIR ${CMAKE_SOURCE_DIR})
message(ABSOLUTE_SCRIPT_PATH="${ABSOLUTE_SCRIPT_PATH}")

add_custom_command(OUTPUT ${RESOLVED_ROS_DIR}/../../build/libInertialSenseSDK.a
    COMMAND ${CMAKE_COMMAND} -E env bash ${ABSOLUTE_SCRIPT_PATH}
    COMMENT "Running custom shell script..."
    VERBATIM
)

add_custom_target(build_InertialSenseSDK DEPENDS ${RESOLVED_ROS_DIR}/../../build/libInertialSenseSDK.a)

add_library(InertialSenseSDK STATIC IMPORTED GLOBAL)
add_dependencies(InertialSenseSDK build_InertialSenseSDK)
set_target_properties(InertialSenseSDK PROPERTIES IMPORTED_LOCATION ${RESOLVED_ROS_DIR}/../../build/libInertialSenseSDK.a)

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
ament_export_include_directories(shared/include)
ament_export_libraries(inertial_sense_ros2)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs std_msgs diagnostic_msgs)

# We don't need the example projects
set(IGNORE_EXAMPLE_PROJECTS 1)

include_directories(
    shared/include
    ${XMLRPC_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ../../src #This line of CMakeList.txt stays in .external file to reference submodule
    ../../src/libusb/libusb
    ../../src/libusb/linux
    ../../src/yaml-cpp
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

add_executable(${PROJECT_NAME}_node
    src/inertial_sense_ros2.cpp
    shared/src/inertial_sense_node.cpp
    shared/src/ParamHelper.cpp
    shared/src/RtkRover.cpp
    shared/src/RtkBase.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs geometry_msgs nav_msgs diagnostic_msgs sensor_msgs std_srvs)
target_link_libraries(${PROJECT_NAME}_node InertialSenseSDK ${YAML_CPP_LIBRARIES} pthread)


ament_add_gtest(test_${PROJECT_NAME}
    src/inertial_sense_ros2.cpp
    shared/src/ParamHelper.cpp
    shared/src/RtkRover.cpp
    shared/src/RtkBase.cpp
    test/test_basic_unit_tests.cpp
    test/test_client_reconnect.cpp
    test/test_communications.cpp
    #test/test_main.cpp
    #test/test_main.h
    test/gtest_helpers.h
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(test_${PROJECT_NAME} "${cpp_typesupport_target}")
ament_target_dependencies(test_${PROJECT_NAME} rclcpp std_msgs geometry_msgs nav_msgs diagnostic_msgs sensor_msgs std_srvs)
target_link_libraries(test_${PROJECT_NAME} InertialSenseSDK ${YAML_CPP_LIBRARIES} pthread)

#link_directories(/home/s/Inertial_Sense/imx/ros2_ws/src/inertial-sense-sdk)
install(TARGETS test_${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

ament_package()
