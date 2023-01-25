
#include <gtest/gtest.h>
#include "test_main.h"
#include "inertial_sense_ros.h"


cTestNode testNode;



TEST(test_main, basic)
{
    printf("We are here!\n");


	bool success = false;
	unsigned int startTimeMs = current_timeMs();
	while(current_timeMs() - startTimeMs < 5000)
	{
		SLEEP_MS(200);
        printf("waiting...\n");
    }

    ASSERT_TRUE( true );

}

void cTestNode::init()
{
    ros::NodeHandle nh;
    sub_wheel_encoder_      = nh.subscribe("msg_wheel_encoder", 1, &cTestNode::cbWheelEncoder, this);
    sub_imu_                = nh.subscribe("pimu", 1, &cTestNode::cbPIMU, this);
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

    printf("Rx wheel encoder\n");
}

void cTestNode::cbPIMU(const inertial_sense_ros::PIMUPtr &pimu)
{

    printf("Rx PIMU\n");
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
