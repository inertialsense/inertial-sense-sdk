
#include "inertial_sense_ros.h"
#include <filesystem>
#include "gtest_helpers.h"
#include <yaml-cpp/yaml.h>

#define PARAM_YAML_FILE "../../../src/inertial-sense-sdk/ros/launch/test_yaml_config.yaml"

#if 1
TEST(ROSCommunicationsTests, basic)
{
    TEST_COUT << "Test Communications" << std::endl;
    bool success = true;
    ASSERT_TRUE( success );
}
#endif

/***
 * This set of tests validates InertialSenseROS publishes the configured topics as DIDs come in from the InertialSense SDK.
 *
 * The main function would bring up an instance of InertialSenseROS with a particular configurations, and starts its running.
 *
 * Each subsequent test brings up the ROS subscriber, and waits for a message to arrive.  If a message arrives before the timeout period,
 * the message contents are evaluated to ensure that the data is "reasonable" (ie, lat/lon are in -180-180, and -90-90, etc.
 * If this passes, the test passes.
 */


bool navsatfix_passed = false;
sensor_msgs::NavSatFix msg_NavSatFix;
void navsatfix_callback(const sensor_msgs::NavSatFixConstPtr& fix) {
    msg_NavSatFix = *fix;
}

TEST(ROSCommunicationsTests, test_navsatfix )
{
    ros::NodeHandle nh;

    nh.subscribe("/NavSatFix", 100, navsatfix_callback);
    double expires = ros::Time::now().toSec() + 5.0;
    while(!navsatfix_passed && (ros::Time::now().toSec() < expires)) {
        SLEEP_MS(200);
        TEST_COUT << "waiting for NavSatFix message...  (timeout in " << (expires - ros::Time::now().toSec()) << " seconds)" << std::endl;

    }

    ASSERT_TRUE(navsatfix_passed);
    EXPECT_GE(msg_NavSatFix.latitude, -90.0);
    EXPECT_LE(msg_NavSatFix.latitude, 90.0);
    EXPECT_GE(msg_NavSatFix.longitude, -180.0);
    EXPECT_LE(msg_NavSatFix.longitude, 180.0);
}

/*
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "ros-communication-tests");
    ros::NodeHandle nh;

    // typical runtime location is <repo-root>/catkin_ws/build/inertial-sense-sdk/ros
    std::ifstream yaml(PARAM_YAML_FILE);
    // ASSERT_FALSE(yaml.fail()) << "Unable to locate or access " << PARAM_YAML_FILE << ".  CWD is " << std::filesystem::current_path();
    YAML::Node config = YAML::Load(yaml);
    // ASSERT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";
    InertialSenseROS isROS(config);

    return RUN_ALL_TESTS();
}
*/
