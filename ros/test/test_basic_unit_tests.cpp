//
// Created by kylemallory on 2/5/23.
//

#include <fstream>
#include <filesystem>

#include "gtest_helpers.h"

#include <yaml-cpp/yaml.h>

#include "../include/inertial_sense_ros.h"

#define PARAM_YAML_FILE "../../../src/inertial-sense-sdk/ros/launch/example_params.yaml"

TEST(BasicTestSuite, test_config_params)
{
    // typical runtime location is <repo-root>/catkin_ws/build/inertial-sense-sdk/ros
    std::ifstream yaml(PARAM_YAML_FILE);
    EXPECT_FALSE(yaml.fail()) << "Unable to locate or access " << PARAM_YAML_FILE << ".  CWD is " << std::filesystem::current_path();

    YAML::Node config = YAML::Load(yaml);
    EXPECT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";
    InertialSenseROS isROS(config);

    // check various parts of the config to ensure the config was read and parameters were parsed and configured properly.
    // This is a very high level test, and is not intended to test every parameter, just a few key ones (of different types, nested, etc).

    // This should also test composite configurations - ie, configuration parameters that when combined, set a single underlying parameter, ie ioConfigBits

    ASSERT_EQ(isROS.port_, "/dev/ttyACM0");
    ASSERT_EQ(isROS.baudrate_, 921600);

    ASSERT_EQ(isROS.rs_.gps1.enabled, true);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);

    // FIXME: Ideally, we shouldn't need to startup ROS to perform basic unit tests, but the InertialSenseROS constructor requires ROS to be running.
    // We should move this to an initROS() function which can be called from a parameterized constructor, and implement a default constructor that
    // initializes class fields/members, but doesn't do anything else.
    ros::init(argc, argv, "test_unit_tests");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}