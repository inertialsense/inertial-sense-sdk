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
    ASSERT_FALSE(yaml.fail()) << "Unable to locate or access " << PARAM_YAML_FILE << ".  CWD is " << std::filesystem::current_path();

    YAML::Node config = YAML::Load(yaml);
    ASSERT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";

    InertialSenseROS isROS(config);

    // check various parts of the config to ensure it was read and parameters were parsed and configured properly.
    // This is a very high level test, and is not intended to test every parameter, just a few key ones (of different types, nested, etc).

    // This should also test composite configurations - ie, configuration parameters that when combined, set a single underlying parameter, ie ioConfigBits

    EXPECT_EQ(isROS.ports_.size(), 1);
    EXPECT_EQ(isROS.ports_[0], "/dev/ttyACM0");
    EXPECT_EQ(isROS.baudrate_, 921600);

    EXPECT_EQ(isROS.rs_.gps1.enabled, true);
}

TEST(BasicTestSuite, test_rtk_rover)
{
    // typical runtime location is <repo-root>/catkin_ws/build/inertial-sense-sdk/ros
    std::ifstream yaml(PARAM_YAML_FILE);
    ASSERT_FALSE(yaml.fail()) << "Unable to locate or access " << PARAM_YAML_FILE << ".  CWD is " << std::filesystem::current_path();

    YAML::Node config = YAML::Load(yaml);
    ASSERT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";

    InertialSenseROS isROS(config);

    // check various parts of the config to ensure the config was read and parameters were parsed and configured properly.
    // This is a very high level test, and is not intended to test every parameter, just a few key ones (of different types, nested, etc).

    // This should also test composite configurations - ie, configuration parameters that when combined, set a single underlying parameter, ie ioConfigBits

    ASSERT_NE(isROS.RTK_rover_, nullptr);
    RtkRoverProvider *p = isROS.RTK_rover_;
    EXPECT_EQ(p->enable, true);
    EXPECT_EQ(p->positioning_enable, true);
    EXPECT_EQ(p->compassing_enable, false);

    ASSERT_NE(p->correction_input, nullptr);
    EXPECT_EQ(p->correction_input->type_, "ntrip");

    RtkRoverCorrectionProvider_Ntrip *ntrip = (RtkRoverCorrectionProvider_Ntrip *)p->correction_input;
    EXPECT_EQ(p->correction_input->protocol_, "RTCM3");
    EXPECT_EQ(ntrip->ip_, "66.219.246.93");
    EXPECT_EQ(ntrip->port_, 7777);
    EXPECT_EQ(ntrip->get_connection_string(), "TCP:RTCM3:66.219.246.93:7777");
    EXPECT_EQ(ntrip->connectivity_watchdog_enabled_, true);

    isROS.configure_rtk();
    EXPECT_EQ(isROS.rtkConfigBits_, 0x402);
}

TEST(BasicTestSuite, test_rtk_base)
{
    // typical runtime location is <repo-root>/catkin_ws/build/inertial-sense-sdk/ros
    std::ifstream yaml(PARAM_YAML_FILE);
    ASSERT_FALSE(yaml.fail()) << "Unable to locate or access " << PARAM_YAML_FILE << ".  CWD is " << std::filesystem::current_path();

    YAML::Node config = YAML::Load(yaml);
    ASSERT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";

    InertialSenseROS isROS(config);

    // check various parts of the config to ensure the config was read and parameters were parsed and configured properly.
    // This is a very high level test, and is not intended to test every parameter, just a few key ones (of different types, nested, etc).

    // This should also test composite configurations - ie, configuration parameters that when combined, set a single underlying parameter, ie ioConfigBits

    ASSERT_NE(isROS.RTK_base_, nullptr);
    RtkBaseProvider *p = isROS.RTK_base_;
    EXPECT_EQ(p->enable, true);
    EXPECT_EQ(p->source_gps__serial0_, RtkBaseProvider::base_gps_source::GPS1);
    EXPECT_EQ(p->source_gps__serial1_, RtkBaseProvider::base_gps_source::OFF);
    EXPECT_EQ(p->source_gps__serial2_, RtkBaseProvider::base_gps_source::GPS2);
    EXPECT_EQ(p->correction_outputs_.size(), 1);

    EXPECT_EQ(p->correction_outputs_[0]->type_, "ntrip");

    RtkBaseCorrectionProvider_Ntrip *ntrip = (RtkBaseCorrectionProvider_Ntrip *)p->correction_outputs_[0];
    EXPECT_EQ(ntrip->protocol_, "RTCM3");
    EXPECT_EQ(ntrip->ip_, "127.0.0.1");
    EXPECT_EQ(ntrip->port_, 7777);
    EXPECT_EQ(ntrip->get_connection_string(), "TCP:RTCM3:127.0.0.1:7777");

    isROS.configure_rtk();
    EXPECT_EQ(isROS.rtkConfigBits_, 0x402);


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