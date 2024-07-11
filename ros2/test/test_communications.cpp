
#include "inertial_sense_ros.h"
#include <filesystem>
#include "gtest_helpers.h"
#include <yaml-cpp/yaml.h>

#define PARAM_YAML_FILE "../../../src/inertial-sense-sdk/ros/launch/test_config.yaml"

class gpsTestNode
{
private:
    ros::Subscriber sub_navsatfix;
public:
    gpsTestNode() {}
    void init() {
        ros::NodeHandle nh;
        sub_navsatfix = nh.subscribe("/NavSatFix", 1, &gpsTestNode::cb_navsatfix, this);
    }

    bool navsatfix_passed = false;
    sensor_msgs::NavSatFix msg_NavSatFix;
    void cb_navsatfix(const sensor_msgs::NavSatFixConstPtr& fix) {
        msg_NavSatFix = *fix;
        navsatfix_passed = true;
        TEST_COUT << "Rx NavSatFix\n";
    }

};

/***
 * This set of tests validates that InertialSenseROS publishes the configured topics with each DID that comes in from the InertialSense SDK.
 *
 * The main function would bring up an instance of InertialSenseROS with a particular configurations, and start its running.
 *
 * Each subsequent test brings up the ROS subscriber, and waits for a message to arrive.  If a message arrives before the timeout period,
 * the message contents are evaluated to ensure that the data is "reasonable" (ie, lat/lon are in -180-180, and -90-90, etc.
 * If this passes, the test passes.
 */

TEST(ROSCommunicationsTests, test_navsatfix )
{
    gpsTestNode testNode;
    testNode.init();

    std::string yaml = "topic: \"inertialsense\"\n"
                       "port: [/dev/ttyACM0, /dev/ttyACM1, /dev/ttyACM2]\n"
                       "baudrate: 921600\n"
                       "\n"
                       "ins:\n"
                       "  navigation_dt_ms: 16                          # EKF update period.  uINS-3: 4  default, 1 max.  Use `msg/ins.../period` to reduce INS output data rate."
                       "\n"
                       "gps1:\n"
                       "  messages:\n"
                       "    navsatfix:\n"
                       "      topic: \"/NavSatFix\"                        # /navsatfix\n"
                       "      enable: true";

    YAML::Node config = YAML::Load(yaml);
    ASSERT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";

    InertialSenseROS isROS(config);
    isROS.initialize();

    double now = ros::Time::now().toSec();
    double expires = now + 5.0, nextMsg = now + 1.0;
    do {
        isROS.update();
        ros::spinOnce();
        SLEEP_MS(100);

        if (now > nextMsg) {
            TEST_COUT << "waiting for gps1/pos_vel message...  (timeout in " << (expires - now) << " seconds)" << std::endl;
            nextMsg = now + 1.0;
        }

        now = ros::Time::now().toSec();
    } while(!testNode.navsatfix_passed && (now < expires));

    EXPECT_TRUE(testNode.navsatfix_passed);
    EXPECT_GE(testNode.msg_NavSatFix.latitude, -90.0);
    EXPECT_LE(testNode.msg_NavSatFix.latitude, 90.0);
    EXPECT_GE(testNode.msg_NavSatFix.longitude, -180.0);
    EXPECT_LE(testNode.msg_NavSatFix.longitude, 180.0);
}
