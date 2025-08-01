//
// Created by kylemallory on 2/5/23.
//

#include <fstream>

#include "gtest_helpers.h"

#include <yaml-cpp/yaml.h>

#include "../include/inertial_sense_ros.h"

#define PARAM_YAML_FILE "../../../../src/inertial-sense-sdk/ROS/ros1/launch/test_config.yaml"

char cwd_buff[256];

TEST(BasicTestSuite, test_config_params)
{
    // typical runtime location is <repo-root>/catkin_ws/build/inertial-sense-sdk/ros
    std::ifstream yaml(PARAM_YAML_FILE);
    ASSERT_FALSE(yaml.fail()) << "Unable to locate or access " << PARAM_YAML_FILE << ".  CWD is " << getcwd(cwd_buff, sizeof(cwd_buff)) << ", errno=" << errno;

    YAML::Node config = YAML::Load(yaml);
    ASSERT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";

    InertialSenseROS isROS(config);

    // check various parts of the config to ensure it was read and parameters were parsed and configured properly.
    // This is a very high level test, and is not intended to test every parameter, just a few key ones (of different types, nested, etc).

    // This should also test composite configurations - ie, configuration parameters that when combined, set a single underlying parameter, ie ioConfigBits

    EXPECT_GE(isROS.ports_.size(), 1);
    EXPECT_EQ(isROS.ports_[0], "/dev/ttyACM0");
    EXPECT_EQ(isROS.baudrate_, 921600);

    EXPECT_EQ(isROS.rs_.gps1.enabled, true);
}

TEST(BasicTestSuite, test_rtk_rover)
{
    // typical runtime location is <repo-root>/catkin_ws/build/inertial-sense-sdk/ros
    std::ifstream yaml(PARAM_YAML_FILE);
    ASSERT_FALSE(yaml.fail()) << "Unable to locate or access " << PARAM_YAML_FILE << ".  CWD is " << getcwd(cwd_buff, sizeof(cwd_buff)) << ", errno=" << errno;

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

    isROS.initialize();
    EXPECT_TRUE(isROS.sdk_connected_) << "Unable to connect to device.";
    nvm_flash_cfg_t flashCfg;
    isROS.IS_.WaitForImxFlashCfgSynced();
    isROS.IS_.ImxFlashConfig(flashCfg);
    EXPECT_EQ(flashCfg.RTKCfgBits, 0x2);
}

TEST(BasicTestSuite, test_rtk_rover_ntrip)
{
    std::string yaml = "rtk_rover:\n"
                       "  correction_input: \n"
                       "    select: ntrip_example                       # Select correction input settings\n"
                       "\n"
                       "    ntrip_example:                              # Rename as needed\n"
                       "      type: ntrip                               # configure NTRIP client provider (RTK Rover = TCP Client)\n"
                       "      format: RTCM3\n"
                       "      ip_address: '66.219.246.93'               # NTRIP service address\n"
                       "      ip_port: 7777                             # NTRIP service port\n"
                       "      mount_point: 'mountabc'                   # NTRIP service mount-point (if any)\n"
                       "      username: 'myusername'                    # NTRIP service username (if any)\n"
                       "      password: 'yourpassword'                  # NTRIP service password (if any)\n"
                       "      connection_attempts:\n"
                       "        limit: 5                                # every time we attempt to connect, only try one time\n"
                       "        backoff: 10                             # default back off, ignored because only connecting once\n"
                       "      watchdog:                                 # connectivity watchdog timer - reconnects if no RTK activity\n"
                       "        enable: true\n"
                       "        interval: 3                             # check every 1 second for traffic\n"
                       "        timeout: 15                             # number of seconds of inactivity before attempting to reconnect\n";

    YAML::Node config = YAML::Load(yaml);
    ASSERT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";

    // check various parts of the config to ensure the config was read and parameters were parsed and configured properly.
    // This is a very high level test, and is not intended to test every parameter, just a few key ones (of different types, nested, etc).

    // This should also test composite configurations - ie, configuration parameters that when combined, set a single underlying parameter, ie ioConfigBits
    RtkRoverProvider provider(config["rtk_rover"]);

    EXPECT_EQ(provider.enable, true);
    ASSERT_NE(provider.correction_input, nullptr);
    EXPECT_EQ(provider.correction_input->type_, "ntrip");

    RtkRoverCorrectionProvider_Ntrip *ntrip = (RtkRoverCorrectionProvider_Ntrip *)provider.correction_input;
    EXPECT_EQ(ntrip->protocol_, "RTCM3");
    EXPECT_EQ(ntrip->ip_, "66.219.246.93");
    EXPECT_EQ(ntrip->port_, 7777);
    EXPECT_EQ(ntrip->mount_point_, "mountabc");
    EXPECT_EQ(ntrip->username_, "myusername");
    EXPECT_EQ(ntrip->password_, "yourpassword");
    EXPECT_EQ(ntrip->get_connection_string(), "TCP:RTCM3:66.219.246.93:7777:mountabc:myusername:yourpassword");
    EXPECT_EQ(ntrip->connection_attempt_limit_, 5);
    EXPECT_EQ(ntrip->connection_attempt_backoff_, 10);
    EXPECT_EQ(ntrip->connectivity_watchdog_enabled_, true);
    EXPECT_EQ(ntrip->connectivity_watchdog_timer_frequency_, 3);
}

TEST(BasicTestSuite, test_rtk_rover_serial)
{
    std::string yaml = "rtk_rover:\n"
                       "  correction_input: \n"
                       "    select: serial_example                      # Select correction input settings\n"
                       "\n"
                       "    serial_example:\n"
                       "      type: serial                              # host/pc serial port, always 8N1 - IS-Ros will open and forward data from the EVB to this port\n"
                       "      format: RTCM3\n"
                       "      port: /dev/ttyUSB0                        # Serial port of local host where corrections can be read from (not the EVB/IMX port!)\n"
                       "      baud_rate: 115200                         # Serial baud rate\n";

    YAML::Node config = YAML::Load(yaml);
    ASSERT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";
    RtkRoverProvider provider(config["rtk_rover"]);

    ASSERT_NE(provider.correction_input, nullptr);
    EXPECT_EQ(provider.correction_input->type_, "serial");

    RtkRoverCorrectionProvider_Serial *serial = (RtkRoverCorrectionProvider_Serial *)provider.correction_input;
    EXPECT_EQ(serial->protocol_, "RTCM3");
    EXPECT_EQ(serial->port_, "/dev/ttyUSB0");
    EXPECT_EQ(serial->baud_rate_, 115200);
}

TEST(BasicTestSuite, test_rtk_rover_ros)
{
    std::string yaml = "rtk_rover:\n"
                       "  correction_input: \n"
                       "    select: ros_provider                        # Select correction input settings\n"
                       "\n"
                       "    ros_provider:\n"
                       "      topic: \"/rtcm3_corrections\"\n"
                       "      format: RTCM3\n"
                       "      type: \"ros_topic\"                       # to/from ROS topics - ROS topic ot subscribe to, received RTCM3 corrections are passed to the EVB\n";

    YAML::Node config = YAML::Load(yaml);
    ASSERT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";
    RtkRoverProvider provider(config["rtk_rover"]);

    ASSERT_NE(provider.correction_input, nullptr);
    EXPECT_EQ(provider.correction_input->type_, "ros_topic");

    RtkRoverCorrectionProvider_ROS *ros = (RtkRoverCorrectionProvider_ROS *)provider.correction_input;
    EXPECT_EQ(ros->protocol_, "RTCM3");
    EXPECT_EQ(ros->topic_, "/rtcm3_corrections");
}

TEST(BasicTestSuite, test_rtk_rover_evb)
{
    std::string yaml = "rtk_rover:\n"
                       "  correction_input: \n"
                       "    select: xbee_radio                          # Select correction input settings\n"
                       "\n"
                       "    xbee_radio:\n"
                       "      type: \"evb\"\n"
                       "      format: RTCM3\n"
                       "      port: \"xbee\"";

    YAML::Node config = YAML::Load(yaml);
    ASSERT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";

    RtkRoverProvider provider(config["rtk_rover"]);

    ASSERT_NE(provider.correction_input, nullptr);
    EXPECT_EQ(provider.correction_input->type_, "evb");

    RtkRoverCorrectionProvider_EVB *evb = (RtkRoverCorrectionProvider_EVB *)provider.correction_input;
    EXPECT_EQ(evb->protocol_, "RTCM3");
    EXPECT_EQ(evb->port_, "xbee");
}

TEST(BasicTestSuite, test_rtk_base)
{
    // typical runtime location is <repo-root>/catkin_ws/build/inertial-sense-sdk/ros
    std::ifstream yaml(PARAM_YAML_FILE);
    ASSERT_FALSE(yaml.fail()) << "Unable to locate or access " << PARAM_YAML_FILE << ".  CWD is " << getcwd(cwd_buff, sizeof(cwd_buff)) << ", errno=" << errno;

    YAML::Node config = YAML::Load(yaml);
    ASSERT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";

    InertialSenseROS isROS(config);

    // check various parts of the config to ensure the config was read and parameters were parsed and configured properly.
    // This is a very high level test, and is not intended to test every parameter, just a few key ones (of different types, nested, etc).

    // This should also test composite configurations - ie, configuration parameters that when combined, set a single underlying parameter, ie ioConfigBits

    ASSERT_NE(isROS.RTK_base_, nullptr);
    RtkBaseProvider *p = isROS.RTK_base_;
    EXPECT_EQ(p->enable, false);
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

    isROS.initialize();
    EXPECT_TRUE(isROS.sdk_connected_) << "Unable to connect to device.";
    nvm_flash_cfg_t flashCfg;
    isROS.IS_.WaitForImxFlashCfgSynced();
    isROS.IS_.ImxFlashConfig(flashCfg);
    EXPECT_EQ(flashCfg.RTKCfgBits, 0x2);
}

TEST(BasicTestSuite, test_topic_helper)
{
    std::string yaml = "ins:\n"
                       "  navigation_dt_ms: 4\n"
                       "  messages:\n"
                       "    odom_ins_enu:\n"
                       "      enable: true\n"
                       "    odom_ins_ned:\n"
                       "      topic: \"odom_ned\"\n"
                       "      enable: false\n"
                       "    odom_ins_ecef:\n"
                       "      topic: \"odom_ins_ecef\"\n"
                       "      period: 2\n"
                       "sensors:\n"
                       "  messages:  \n"
                       "    imu:              # Publish IMU angular rates and linear acceleration\n"
                       "    pimu:             # Publish preintegrated IMU delta theta and delta velocity\n"
                       "    barometer:\n";
                    // "    magnetometer:\n" -- these should test to "enable: false"
                    // "    strobe_in:       -- these should test to "enable: false"

    YAML::Node config = YAML::Load(yaml);
    ASSERT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";
    ParamHelper ph(config);

    // Check to ensure that parsing of configs using ParamHelper+TopicHelper work as expected.

    YAML::Node insNode = ph.node(config, "ins");
    YAML::Node insMsgs = ph.node(insNode, "messages", 2);
    // These first set of tests on the 'ins' stanza test that explicit and default parameters are working.

    TopicHelper enu;
    ph.msgParams(enu, "odom_ins_enu");
    EXPECT_EQ(enu.enabled, true);
    EXPECT_EQ(enu.topic, "odom_ins_enu");
    EXPECT_EQ(enu.period, 1);

    TopicHelper ned;
    ph.msgParams(ned, "odom_ins_ned");
    EXPECT_EQ(ned.enabled, false);
    EXPECT_EQ(ned.topic, "odom_ned");
    EXPECT_EQ(ned.period, 1);

    TopicHelper ecef;
    ph.msgParams(ecef, "odom_ins_ecef");
    EXPECT_EQ(ecef.enabled, true);
    EXPECT_EQ(ecef.topic, "odom_ins_ecef");
    EXPECT_EQ(ecef.period, 2);

    YAML::Node sensorsNode = ph.node(config, "sensors");
    YAML::Node sensorsMsgs = ph.node(sensorsNode, "messages", 2);
    // These next set of tests on the 'sensors' stanza test that implicit (and empty/null) mappings
    // default to appropriate "enabled" values when the
    // Specifically, that 'sensors/messages' stanza above, each empty/null sub-stanza (imu, pimu, etc)
    // has an implicit default topic and enabled setting.

    TopicHelper imu;
    ph.msgParams(imu, "imu");
    EXPECT_EQ(imu.enabled, true);
    EXPECT_EQ(imu.topic, "imu");
    EXPECT_EQ(imu.period, 1);

    TopicHelper pimu;
    ph.msgParams(pimu, "pimu");
    EXPECT_EQ(pimu.enabled, true);
    EXPECT_EQ(pimu.topic, "pimu");
    EXPECT_EQ(pimu.period, 1);

    TopicHelper baro;
    ph.msgParams(baro, "barometer");
    EXPECT_EQ(baro.enabled, true);
    EXPECT_EQ(baro.topic, "barometer");
    EXPECT_EQ(baro.period, 1);

    TopicHelper mag;
    ph.msgParams(mag, "magnetometer");
    EXPECT_EQ(mag.enabled, false); // NOTE: this is false, because 'magnetometer' doesn't appear in the YAML, and its not implictly enabled
    EXPECT_EQ(mag.topic, "magnetometer");
    EXPECT_EQ(mag.period, 1);

    TopicHelper strobe;
    ph.msgParams(strobe, "strobe_in", "strobe", true); // NOTE: This is NOT enabled, despite being enabled, but the stanza doesn't exist
    EXPECT_EQ(strobe.enabled, false); // NOTE: this is FALSE, despite `enabledDefault: true`, because "strobe_in" does not exist in the YAML (implicitly disabled)
    EXPECT_EQ(strobe.topic, "strobe");
    EXPECT_EQ(strobe.period, 1);

    ph.msgParamsImplicit(strobe, "strobe_in", "strobe", true); // NOTE: This overrides the default implicit behavior (implicitly enabled)
    EXPECT_EQ(strobe.enabled, true); // NOTE: this is TRUE, despite not being in the YAML, because we called msgParamsImplicit(..., enabledDefault: true)
    EXPECT_EQ(strobe.topic, "strobe");
    EXPECT_EQ(strobe.period, 1);
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