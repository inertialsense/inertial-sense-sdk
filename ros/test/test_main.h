#pragma once

// #define HRPTUNING

#include <termios.h>
#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "inertial_sense_ros.h"


class cTestNode
{
public:
    cTestNode(){}
    void init();
    bool step();
    void cbWheelEncoder(const sensor_msgs::JointState &msg);
    void cbPIMU(const inertial_sense_ros::PIMUPtr &pimu);

    bool did_rx_pimu_ = false;

private:

    ros::Subscriber sub_wheel_encoder_;    
    ros::Subscriber sub_pimu_;
    

};
