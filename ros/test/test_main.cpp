#include "gtest_helpers.h"
#include "test_main.h"
#include "inertial_sense_ros.h"

cTestNode testNode;

TEST(test_main, basic)
{
    std::string yaml = "topic: \"inertialsense\"\n"
                       "port: [/dev/ttyACM0, /dev/ttyACM1]\n"
                       "baudrate: 921600\n"
                       "ioConfig: 0x026B2060\n"
                       "\n"
                       "ins:\n"
                       "  navigation_dt_ms: 16                          # EKF update period.  uINS-3: 4  default, 1 max.  Use `msg/ins.../period` to reduce INS output data rate."
                       "\n"
                       "sensors:\n"
                       "  messages:  \n"
                       "    pimu:             # Publish preintegrated IMU delta theta and delta velocity\n"
                       "      topic: \"pimu\"\n"
                       "      enable: true\n"
                       "      period: 1\n";

    YAML::Node config = YAML::Load(yaml);
    ASSERT_TRUE(config.IsDefined()) << "Unable to parse YAML file. Is the file valid?";

    InertialSenseROS isROS(config);
    isROS.initialize();

    bool success = false;
    unsigned int startTimeMs = current_timeMs(), prevTimeMs = 0, nowTimeMs;
	while((nowTimeMs = current_timeMs()) - startTimeMs < 5000)
	{
        isROS.update();
        if (testNode.did_rx_pimu_) {
            success = true;
            break;
        } else {
            // check regularly, but don't print regularly..
            SLEEP_MS(200);
            if (prevTimeMs / 1000 != nowTimeMs / 1000) {
                TEST_COUT << "waiting...  (time: " << nowTimeMs << ")" << std::endl;
                prevTimeMs = nowTimeMs;
            }
        }
    }

    ASSERT_TRUE( success );
    isROS.terminate();
}

void cTestNode::init()
{
    ros::NodeHandle nh;
    sub_wheel_encoder_      = nh.subscribe("msg_wheel_encoder", 1, &cTestNode::cbWheelEncoder, this);
    sub_pimu_               = nh.subscribe("pimu", 1, &cTestNode::cbPIMU, this);
}

bool cTestNode::step()
{
    // static double last_time = 0;
    // if( ros::ok() )
    // {
    //     double current_time = ros::Time::now().toSec();
    //     double dt = current_time - last_time;
    //     last_time = current_time;
    //     ros::spinOnce();
    //     current_time = ros::Time::now().toSec();
    //     ros::Rate rate(50);     // 50 ms
    //     rate.sleep();
    //     dt = ros::Time::now().toSec() - current_time;

    //     return true;
    // }

    return false;
}

void cTestNode::cbWheelEncoder(const sensor_msgs::JointState &msg)
{
    sensor_msgs::JointState encoder;
    TEST_COUT << "Rx wheel encoder\n";
}

void cTestNode::cbPIMU(const inertial_sense_ros::PIMUPtr &pimu)
{
    testNode.did_rx_pimu_ = true;
    TEST_COUT << "Rx PIMU\n";
}


int main(int argc, char** argv) 
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_inertial_sense_ros");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    testNode.init();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}
