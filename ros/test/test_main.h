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
    void cbPIMU(const inertial_sense_ros::PIMU &pimu);
    void cbIMU(const  sensor_msgs::Imu &imu);
    void cbINS(const nav_msgs::Odometry &ins);
    void cbGPS(const inertial_sense_ros::GPS &gps);

    int get_deviations(std::vector<double> &a, std::vector<double> &b, std::vector<double> &out);
    double get_avg_deviation(std::vector<double> &a, std::vector<double> &b);
    double get_min_deviation(std::vector<double> &a, std::vector<double> &b);
    double get_max_deviation(std::vector<double> &a, std::vector<double> &b);

    bool quiet = true;
    bool got_gps_tow = false;
    bool did_rx_pimu_ = false;

    std::vector<double> gps_ts;
    std::vector<double> imu_ts;
    std::vector<double> pimu_ts;
    std::vector<double> ins_ts;

private:

    ros::Subscriber sub_wheel_encoder_;    
    ros::Subscriber sub_pimu_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_gps1_;
    ros::Subscriber sub_ins_;


};
