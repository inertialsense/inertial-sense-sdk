
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "gtest_helpers.h"
#include "inertial_sense_ros.h"

#define PARAM_YAML_FILE "../../../src/inertial-sense-sdk/ros/launch/test_config.yaml"

class gpsTestNode: public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_navsatfix;
public:

    //rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("nh");
    gpsTestNode() : Node("nh_gps") {}
    void init() {

        sub_navsatfix = this->create_subscription<sensor_msgs::msg::NavSatFix>("/NavSatFix", 1, std::bind(&gpsTestNode::cb_navsatfix, this, std::placeholders::_1));

    }

    bool navsatfix_passed = false;
    sensor_msgs::msg::NavSatFix msg_NavSatFix;
    void cb_navsatfix(const sensor_msgs::msg::NavSatFix::SharedPtr fix) {
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

TEST(ROSCommunicationsTests, test_navsatfix ) {
    auto testNode = std::make_shared<gpsTestNode>();
    testNode->init();

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

    double now = testNode->now().seconds();
    //auto testNode = std::make_shared<gpsTestNode>();
    double expires = now + 150.0, nextMsg = now + 1.0;
    do {
        isROS.update();
        rclcpp::spin_some(testNode);
        SLEEP_MS(100);

        if (now > nextMsg) {
            TEST_COUT << "waiting for NavSatFix message...  (timeout in " << (expires - now) << " seconds)" << std::endl;
            nextMsg = now + 1.0;
        }

        now = testNode->now().seconds();
    } while(!testNode->navsatfix_passed && (now < expires));

    EXPECT_TRUE(testNode->navsatfix_passed);
    EXPECT_GE(testNode->msg_NavSatFix.latitude, -90.0);
    EXPECT_LE(testNode->msg_NavSatFix.latitude, 90.0);
    EXPECT_GE(testNode->msg_NavSatFix.longitude, -180.0);
    EXPECT_LE(testNode->msg_NavSatFix.longitude, 180.0);
}
